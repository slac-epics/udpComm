/* $Id: padStream.c,v 1.2 2009/12/15 23:28:39 strauman Exp $ */

#include <udpComm.h>
#include <padProto.h>
#include <lanIpBasic.h>
#include <drvLan9118.h>
#include <rtems.h>
#include <rtems/timerdrv.h>
#include <errno.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include <netinet/in.h> /* for ntohs & friends */

#define NCHNS	4	/* channels on a PAD */

#include <padStream.h>

#include "bpmsim.h"

int padStreamDebug = 0;

time_t padStreamTimeoutSecs = 60;

time_t padStreamPetTime;

/* Data stream implementation. This could all be done over the
 * udpSock abstraction but less efficiently since we would have
 * to copy the PAD fifo to memory first instead of copying the
 * PAD fifo to the lan9118 chip directly...
 */

typedef struct StreamPeerRec_ {
	uint32_t  ip;
	uint16_t  port;
} StreamPeerRec, *StreamPeer;

static StreamPeerRec peer = { 0, 0 };


/* The padProtoHandler and the thread sending the data
 * both access the reply data structure;
 */
static rtems_id mutex = 0;

#define LOCK() \
	assert( RTEMS_SUCCESSFUL == rtems_semaphore_obtain(mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT) )

#define UNLOCK() \
	assert( RTEMS_SUCCESSFUL == rtems_semaphore_release(mutex) )

static LanIpPacketRec replyPacket __attribute__((aligned(4))) = {{{{{0}}}}};
static int            nsamples; /* keep value around (lazyness) */

static IpBscIf intrf;
static int (*start_stop_cb)(int start, void *uarg) = 0;
static void  *cbarg = 0;

static volatile int isup = 0;

int
padStreamInitialize(void *if_p, int (*cb)(int start, void *uarg), void *uarg)
{
rtems_status_code sc;
	
	sc = rtems_semaphore_create(
			rtems_build_name('p','a','d','S'),
			1,
            RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
			0,
			&mutex);

	if ( RTEMS_SUCCESSFUL != sc ) {
		mutex = 0;
		return sc;
	}
	intrf         = if_p;
	start_stop_cb = cb;
	cbarg         = uarg;
	return 0;
}

int
padStreamCleanup()
{
	rtems_semaphore_delete(mutex);
	mutex = 0;
	intrf = 0;
	return 0;
}

/* refresh timestamp and transaction id */
static void
dopet(PadRequest req, PadReply rply)
{
struct timeval now;
	rply->timestampHi = req->timestampHi;
	rply->timestampLo = req->timestampLo;
	rply->xid         = req->xid;
	gettimeofday(&now, 0);
	padStreamPetTime = now.tv_sec;
}

int
padStreamStart(PadRequest req, PadStrmCommand scmd, int me, uint32_t hostip)
{
PadReply        rply = &lpkt_udp_pld(&replyPacket, PadReplyRec);
int             len;
int             rval;


	if ( !intrf )
		return -ENODEV; /* stream has not been initialized yet */


	if ( (nsamples = ntohl(scmd->nsamples)) > 720/4  ) {
		/* doesn't fit in one packet */
		return -EINVAL;
	}

	LOCK();

		if ( ! peer.ip ) {
				peer.ip   = hostip;
				peer.port = ntohs(scmd->port);
		} else {
			/* can only stream to one peer */
			if ( peer.ip != hostip || peer.port != ntohs(scmd->port) ) {
				UNLOCK();
				return -EADDRINUSE;
			}
		}
	
		if ( !isup ) {
			/* Avoid ARP lookup, don't provide destination IP yet */
			udpSockHdrsInitFromIf(intrf, &lpkt_udp_hdrs(&replyPacket), 0, ntohs(scmd->port), ntohs(scmd->port), 0); 	

			/* Add missing bits: destination IP , source port */
			lpkt_ip(&replyPacket).dst     = hostip;

			len = nsamples*sizeof(int16_t)*NCHNS + sizeof(*rply);
			/* Fill in length and IP header checksum etc */
			udpSockHdrsSetlen(&lpkt_udp_hdrs(&replyPacket), len);
			/* Setup Reply */
			rply->version         = req->version;
			rply->type            = PADCMD_STRM | PADCMD_RPLY;
			rply->chnl            = me;
			rply->nBytes          = htons(len);
			rply->status          = 0;
			rply->strm_cmd_flags  = scmd->flags;
			rply->strm_cmd_idx    = 0;

			isup                  = 1;
		}

		dopet(req, rply);


		rval =  start_stop_cb ? start_stop_cb(1, cbarg) : 0;

		if ( rval ) {
			isup      = 0;
			peer.ip   = 0;
			peer.port = 0;
		}

	UNLOCK();

	return rval;
}

int
padStreamPet(PadRequest req, uint32_t hostip)
{
PadReply  rply = &lpkt_udp_pld(&replyPacket, PadReplyRec);
int       err;
	LOCK();
		if ( ! peer.ip ) {
			err = -ENOTCONN;
		} else if ( peer.ip != hostip ) {
			err = -EADDRINUSE;
		} else {
			err = 0;
			dopet(req, rply);
		}
	UNLOCK();
	return err;
}



int bigEndian()
{
union {
	uint8_t  x[2];
	uint16_t tst;
} endian = { x : {0xBE, 0} };
	return(endian.tst == 0xBE00);
}

static inline int16_t
swap(int16_t x)
{
	return x<<8 | ((uint16_t)x)>>8;
}

static void *
streamTest(void *packetBuffer,
			int idx,
			int nsamples,
			int little_endian,
			int column_major,
			void *uarg)
{
int16_t *buf = packetBuffer;
int i;
	if ( !little_endian != bigEndian() ) {
		if ( column_major ) {
			for (i=0; i<nsamples; i++) {
				buf[i*4+0] = swap(i+0x00 + (idx<<8));
				buf[i*4+1] = swap(i+0x10 + (idx<<8));
				buf[i*4+2] = swap(i+0x20 + (idx<<8));
				buf[i*4+3] = swap(i+0x30 + (idx<<8));
			}
		} else {
			for (i=0; i<nsamples; i++) {
				buf[i+0*nsamples] = swap(i+0x00 + (idx<<8));
				buf[i+1*nsamples] = swap(i+0x10 + (idx<<8));
				buf[i+2*nsamples] = swap(i+0x20 + (idx<<8));
				buf[i+3*nsamples] = swap(i+0x30 + (idx<<8));
			}
		}
	} else {
		#define swap(x)	(x)
		if ( column_major ) {
			for (i=0; i<nsamples*NCHNS; i+=4) {
				buf[i+0] = swap(i+0x00 + (idx<<8));
				buf[i+1] = swap(i+0x10 + (idx<<8));
				buf[i+2] = swap(i+0x20 + (idx<<8));
				buf[i+3] = swap(i+0x30 + (idx<<8));
			}
		} else {
			for (i=0; i<nsamples; i++) {
				buf[i+0*nsamples] = swap(i+0x00 + (idx<<8));
				buf[i+1*nsamples] = swap(i+0x10 + (idx<<8));
				buf[i+2*nsamples] = swap(i+0x20 + (idx<<8));
				buf[i+3*nsamples] = swap(i+0x30 + (idx<<8));
			}
		}
		#undef swap
	}
	return buf;
}

static PadStripSimValRec strips;

void *
padStreamSim_getdata(void *packetBuffer,
			int idx,
			int nsamples,
			int little_endian,
			int column_major,
			void *uarg)
{
int16_t		*buf = packetBuffer;
PadStripSimVal ini  = uarg;
int         swp;
static unsigned long noise = 1;

	swp    = ( bigEndian() != !little_endian );

	if ( column_major ) {
		iir2_bpmsim(buf++, nsamples, ini->a, 0,  &noise, swp, NCHNS);
		iir2_bpmsim(buf++, nsamples, ini->b, 0,  &noise, swp, NCHNS);
		iir2_bpmsim(buf++, nsamples, ini->c, 0,  &noise, swp, NCHNS);
		iir2_bpmsim(buf++, nsamples, ini->d, 0,  &noise, swp, NCHNS);
	} else {
		iir2_bpmsim(buf, nsamples, ini->a, 0,  &noise, swp, 1);
		buf += nsamples;
		iir2_bpmsim(buf, nsamples, ini->b, 0,  &noise, swp, 1);
		buf += nsamples;
		iir2_bpmsim(buf, nsamples, ini->c, 0,  &noise, swp, 1);
		buf += nsamples;
		iir2_bpmsim(buf, nsamples, ini->d, 0,  &noise, swp, 1);
	}

	return packetBuffer;
}

uint32_t maxStreamSendDelay1 = 0;
uint32_t maxStreamSendDelay2 = 0;

#ifdef __mcf5200__
#include "hwtmr.h"

extern uint32_t drvLan9118RxIntBase;

#endif

int
padStreamSend(void * (*getdata)(void *packBuffer, int idx, int nsamples, int endianLittle, int colMajor, void *uarg), int type, int idx, void *uarg)
{
int            rval = 0;
PadReply       rply = &lpkt_udp_pld(&replyPacket, PadReplyRec);
DrvLan9118_tps plan = lanIpBscIfGetDrv(intrf);
int            len;
void          *data_p;
uint32_t       now;
struct timeval now_tv;

	if ( idx != 0 )
		return -ENOTSUP;	/* not supported yet */

#ifdef __mcf5200__
	now = Read_hwtimer() - drvLan9118RxIntBase;
	if ( now > maxStreamSendDelay1 )
		maxStreamSendDelay1 = now;
#endif

	LOCK();

	if ( !isup ) {
		UNLOCK();
		return -EAGAIN;
	}

	gettimeofday(&now_tv, 0);
	if ( (now_tv.tv_sec - padStreamPetTime) > padStreamTimeoutSecs ) {
		padStreamStop(0);
		UNLOCK();
		return -ETIMEDOUT;
	}

	/* just look in the cache - we rely on the RX daemon refreshing it */
	if ( (rval = arpLookup(intrf, lpkt_ip(&replyPacket).dst, lpkt_eth(&replyPacket).dst, 1)) ) {
		UNLOCK();
		return rval;
	}

	if ( padStreamDebug & 1 ) {
		/* hack timestamp; tag with our us clock */
		rply->timestampLo = htonl(Read_hwtimer());
	}

	rply->strm_cmd_idx    = idx;
	rply->strm_cmd_flags &= ~PADRPLY_STRM_FLAG_TYPE_SET(-1);
	rply->strm_cmd_flags |=  PADRPLY_STRM_FLAG_TYPE_SET(type);

	len = UDPPKTSZ(ntohs(rply->nBytes));

	if ( drvLan9118TxPacket(plan, 0, len, 0) ) {
		UNLOCK();
		return -ENOSPC;
	}

	drvLan9118FifoWr(plan, &replyPacket, UDPPKTSZ(sizeof(PadReplyRec)));

	if ( (data_p=getdata(
					&rply->data,
					idx,
					nsamples, 
					rply->strm_cmd_flags & PADCMD_STRM_FLAG_LE,
					rply->strm_cmd_flags & PADCMD_STRM_FLAG_CM,
					uarg)) ) {
		drvLan9118FifoWr(plan, data_p, nsamples*NCHNS*sizeof(int16_t));
	}
	/* else ['getdata' returned NULL] the getdata method already
	 * wrote to the TX FIFO
	 */

	drvLan9118TxUnlock(plan);

	UNLOCK();

#ifdef __mcf5200__
	now = Read_hwtimer() - drvLan9118RxIntBase;
	if ( now > maxStreamSendDelay2 )
		maxStreamSendDelay2 = now;
#endif

	return 0;
}

int
padStreamTest()
{
	return padStreamSend(streamTest, 0, 0,0);
}

int
padStreamSim(PadSimCommand scmd, uint32_t hostip)
{
int dosend = 1;
int err;

	if ( !intrf )
		return -ENODEV; /* stream has not been initialized yet */

	if ( scmd ) {
		LOCK();
			if ( ! peer.ip ) {
				err = -ENOTCONN;
			} else if ( hostip && peer.ip != hostip ) {
				err = -EADDRINUSE;
			} else {
				strips.a = ntohl(scmd->a);
				strips.b = ntohl(scmd->b);
				strips.c = ntohl(scmd->c);
				strips.d = ntohl(scmd->d);
				err = 0;
			}
		UNLOCK();

		if ( err )
			return err;

		dosend =  ! (PADCMD_SIM_FLAG_NOSEND & scmd->flags );
	}

	return  dosend ? padStreamSend(padStreamSim_getdata, 0, 0, &strips) : 0;
}

int
padStreamStop(uint32_t hostip)
{
int rval = 0;

	if ( !intrf )
		return -ENODEV; /* stream has not been initialized yet */

	LOCK();
	if ( isup ) {
		if ( !peer.ip ) {
			/* not running */
			rval = -ENOTCONN;
		} else {
			if ( hostip && hostip != peer.ip ) {
				rval = -EACCES;
			} else {
				if ( start_stop_cb )
					rval = start_stop_cb(0, cbarg);

				isup      = 0;
				peer.ip   = 0;
				peer.port = 0;
			}
		}
	} else {
		rval = -ENOTCONN;
	}
	UNLOCK();
	return rval;
}
