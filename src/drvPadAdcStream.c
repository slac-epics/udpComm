/* $Id$ */

#include <rtems.h>
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <lanIpBasicSetup.h>
#include <drvLan9118.h>

#include <epicsThread.h>
#include <errlog.h>
#include <osiSock.h>

#include <drvPadAdcStream.h>

/* Align up to next multiple of 8.
 * We strictly only need multiples of 4 (4 channels)
 * but making sure we always have an even number of
 * samples makes storing the row-major format easier
 */
#define CHANNEL_NO_ALIGN(x)	(((x)+7) & ~7)

/* Just a name for sake of readability */
#define	CS1					1

typedef struct {
	int      channel;
	uint32_t mcaddr;
} *PadUdpThreadArg;

/* Stuff below should eventually move to a separate module */
#include <drvLan9118.h>
#include <lanIpBasic.h>
#include <lanIpBasicSetup.h>
#include <padStream.h>

#undef DEBUG

/* Direct access to the LAN9118 FIFO -- an ugly hack
 * but it's much faster...
 */
#define LAN9118_FIFO_HACK	((volatile void*)0x31000800)

/* Which chip select we are using - this is setup by the BSP
 * but not published :-(
 */
#define CS1_BASE_ADDR		0x30000000


typedef  int16_t  __attribute__ ((may_alias))  int16_t_a;
typedef uint32_t  __attribute__ ((may_alias)) uint32_t_a;

/*
 * /CS (chip-select) of the 4 fifos are driven by A2 / A3.
 * 
 * This leads to the following memory layout starting at CS1_BASE_ADDR:
 *
 *            0..3     channel 0
 *            4..7     channel 1
 *            8..11    channel 2
 *           12..15    channel 3
 *           16..19    channel 0
 *           20..23    channel 1
 *           24..27    channel 2
 *           28..31    channel 3
 *           ... and so on ...
 *
 * Seen as a struct of 16-bit values this looks like this
 *
 * struct {
 *		signed short	ch0;
 *		signed short	ch0;
 *		signed short	ch1;
 *		signed short	ch1;
 *		signed short	ch2;
 *		signed short	ch2;
 *		signed short	ch3;
 *		signed short	ch3;
 * }
 */

static inline
uint32_t	bswap(uint32_t x)
{
	asm volatile("byterev %0; swap %0":"+d"(x));
	return x;
}

#define CPYW(i)      *dst_p++ = src_p[i];
#define CPY4W  do { CPYW(0); CPYW(2); CPYW(4); CPYW(6); } while (0)

#define CPYWS(i)     *dst_p++ = bswap(src_p[i]);
#define CPY4WS do { CPYWS(0); CPYWS(2); CPYWS(4); CPYWS(6); } while (0)

void
drvPadReadFifosColMajor(volatile void *dst_p_v, int idx, int channels, int n, int do_bswap)
{
volatile int16_t_a *dst_p = dst_p_v;
volatile int16_t_a *src_p = (volatile int16_t_a *)CS1_BASE_ADDR;

	assert ( (n & 3) == 0 );

	/* Col-major supported only for all channels ATM */
	assert ( PADCMD_STRM_CHANNELS_ALL == channels );

	n *= PADRPLY_STRM_NCHANNELS;

	if ( 0 == idx ) {
		/* Every channel needs 3 initial clock cycles (Ron uses /CS for the clock also) */
		*(volatile uint32_t_a*)(src_p + 0); src_p[0];
		*(volatile uint32_t_a*)(src_p + 2); src_p[2];
		*(volatile uint32_t_a*)(src_p + 4); src_p[4];
		*(volatile uint32_t_a*)(src_p + 6); src_p[6];
	}

	if ( ! do_bswap ) {
		while ( n >= 8 ) {
			CPY4W;
			CPY4W;
			n-=8;
		}

		/* n must be multiple of 4 */
		while ( n > 0 ) {
			CPY4W;
			n-=4;
		}
	} else {
		while ( n >= 8 ) {
			CPY4WS;
			CPY4WS;
			n-=8;
		}

		/* n must be multiple of 4 */
		while ( n > 0 ) {
			CPY4WS;
			n-=4;
		}
	}
}

#define CPYL       *dst2_p++ = *(volatile uint32_t_a*)src_p;
#define CPY4L  do { CPYL; CPYL; CPYL; CPYL; } while (0)

#define CPYLS      *dst2_p++ = bswap(*(volatile uint32_t_a*)src_p);
#define CPY4LS do { CPYLS; CPYLS; CPYLS; CPYLS; } while (0)

static void
readChannel(volatile int16_t_a *dst_p, volatile int16_t_a *src_p, int leading, int n, int do_bswap)
{
/* word align destination */
register volatile uint32_t_a *dst2_p;

	/* 'leading' indicates that the destination is unaligned */
	if ( leading )
		*dst_p++ = do_bswap ? bswap(*src_p) : *src_p;


	/* Now we are dword aligned and can transfer 32-bit words at a time;
	 * this should work endian-independently :-): the first word read
	 * out of the fifo gets to whatever word-address that is written first...
	 */
	dst2_p = (uint32_t_a*)dst_p;

	if ( !do_bswap ) {
		while ( n>=16 ) {
			CPY4L;
			CPY4L;
			n-=16;
		}

		while ( n>1 ) {
			CPYL;
			n-=2;
		}
	} else {
		while ( n>=16 ) {
			CPY4LS;
			CPY4LS;
			n-=16;
		}

		while ( n>1 ) {
			CPYLS;
			n-=2;
		}
	}

	/* cleanup unaligned end */
	if ( n ) {
		dst_p = (int16_t_a*)dst2_p;
		*dst_p++ = do_bswap ? bswap(*src_p) : *src_p;
	}
}

void
drvPadReadFifosRowMajor(volatile void *dst_pv, int idx, int ch, int n, int do_bswap)
{
volatile int16_t_a *dst_p = dst_pv;
volatile int16_t_a *src_p = (volatile int16_t_a *)CS1_BASE_ADDR;
int                odd, nxt_odd;

	/* Every channel needs 3 initial clock cycles (Ron uses /CS for the clock also) */

	if ( 0 == idx ) {
		if ( ch & 1 )
			*(volatile uint32_t_a*)(src_p + 0); src_p[0];
		if ( ch & 2 )
			*(volatile uint32_t_a*)(src_p + 2); src_p[2];
		if ( ch & 4 )
			*(volatile uint32_t_a*)(src_p + 4); src_p[4];
		if ( ch & 8 )
			*(volatile uint32_t_a*)(src_p + 6); src_p[6];
	}

	/* If n is odd then the destination for the first sample of odd-numbered
	 * channels is not aligned. Get a single sample on the odd channels
	 * and a pair on the even ones.
	 */
	odd = n & 1;
	nxt_odd = 0;

	if ( ch & 1 ) {
		readChannel(dst_p, src_p + 0, nxt_odd, n, do_bswap);
		dst_p += n;
		nxt_odd ^= odd;
	}
	if ( ch & 2 ) {
		readChannel(dst_p, src_p + 2, nxt_odd, n - odd, do_bswap);
		dst_p += n;
		nxt_odd ^= odd;
	}
	if ( ch & 4 ) {
		readChannel(dst_p, src_p + 4, nxt_odd, n, do_bswap);
		dst_p += n;
		nxt_odd ^= odd;
	}
	if ( ch & 8 ) {
		readChannel(dst_p, src_p + 6, nxt_odd, n - odd, do_bswap);
	}
}

void *
drvPadAdcStream_getdata(void *packBuffer, int idx, int channels, int nsamples, int d32, int endianLittle, int colMajor, void *uarg)
{
	assert ( 0 == ( nsamples & 1 ) );
	if ( PADRPLY_STRM_NCHANNELS == channels ) {
		if ( colMajor )
			drvPadReadFifosColMajor( LAN9118_FIFO_HACK, idx, PADCMD_STRM_CHANNELS_ALL, nsamples, endianLittle );	
		else
			drvPadReadFifosRowMajor( LAN9118_FIFO_HACK, idx, PADCMD_STRM_CHANNELS_ALL, nsamples, endianLittle );	
	} else {
			drvPadReadFifosRowMajor( LAN9118_FIFO_HACK, idx, (1<<channels), nsamples, endianLittle );	
	}
	return 0;
}


void
padUdpThread(void *arg);

long 
drvPadAdcStreamReport(int level)
{
	epicsPrintf("PAD Stream Support for Communication with BPM Processor\n");
	epicsPrintf("over 2nd Ethernet Interface\n");
	if ( level > 1 ) {
		epicsPrintf("Dumping Interface Statistics\n");
		drvLan9118DumpStats(lanIpBscIfGetDrv(lanIpIf), 0);
	}
	return 0;
}

int
drvPadAdcStream_start_stop_cb(PadRequest req, PadStrmCommand start, void *uarg)
{
	if ( PADPROTO_VERSION3 != req->version ) {
		/* unrecognized version */
		return -ENOTSUP;
	} 

	if ( start ) {
		if (start->flags & PADCMD_STRM_FLAG_32) {
			/* unsupported */
			return -ENOTSUP;
		}
	}
	return 0;
}
	
epicsThreadId
drvPadAdcStreamInit(int (*start_stop_cb)(PadRequest, PadStrmCommand, void*))
{
char            *padslot, *mcgrp;
PadUdpThreadArg arg  = 0;
epicsThreadId   rval = 0;
struct in_addr  ina;

	if  ( padStreamInitialize(lanIpIf, start_stop_cb, 0) ) {
		return 0;
	}

	if ( ! (arg = malloc(sizeof(*arg))) ) {
		epicsPrintf("drvPadAdcStreamInit(): no memory\n");
		return 0;
	}

	if ( !(padslot = getenv("PADSLOT")) ) {
		epicsPrintf("Unable to start UDP listener; enviroment variable PADSLOT not set\n");
		goto bail;
	}
	arg->channel = atoi(padslot);

	arg->mcaddr  = 0;

	if ( !(mcgrp = getenv("PADMCGRP")) ) {
		epicsPrintf("Warning: 'PADMCGRP' envvar unset; expecting to use BROADCAST for IOC communication\n");
	} else {
		if ( hostToIPAddr(mcgrp, &ina) ) {
			epicsPrintf("Error: Lookup for '%s' failed -- unable to start UDP listener\n", mcgrp);
			goto bail;
		}
		arg->mcaddr = ina.s_addr;
		epicsPrintf("UDP Listener uses IP Multicast (%s) -- reception of broadcast traffic DISABLED\n", mcgrp);
		drvLan9118BcFilterSet(lanIpBscIfGetDrv(lanIpIf), 1);
	}

	epicsPrintf("Starting UDP listener for PAD channel %i\n", arg->channel);
   	rval = epicsThreadCreate(
			"padUdpListener",
			epicsThreadPriorityScanHigh,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			padUdpThread,
			arg);

bail:
	if ( !rval )
		free(arg);
	return rval;
}

void
padUdpThread(void *arg_p)
{
int              err;
PadUdpThreadArg  arg = arg_p;

	/* HACK: must wait for iocInit() to finish -- otherwise
         * set/get RARM might not work (deadlocks experience!)
	 * Just delay...
	 */ 
	epicsThreadSleep(3.0);

	/* use default port and channel number passed as arg */
	if ( (err = padUdpHandler(arg->mcaddr, 0, arg->channel, 1000/*ms*/, 0, 0)) ) {
		epicsPrintf("ERROR: padUdpHandler() aborted: %s\n", strerror(-err));
	}
	/* arg is probably never freed but who cares... */
	free(arg);
}
