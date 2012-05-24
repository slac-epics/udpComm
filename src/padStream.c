/* $Id: padStream.c,v 1.21 2012/05/16 01:39:28 strauman Exp $ */

#include <udpComm.h>
#include <padProto.h>
#include <lanIpBasic.h>
#include <drvLan9118.h>
#include <rtems.h>
#include <rtems/timerdrv.h>
#include <errno.h>
#include <assert.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>

#include <netinet/in.h> /* for ntohs & friends */

#include <padStream.h>

#include "bpmsim.h"

#define LD_FRAG_SIZE 10

int padStreamDebug = 0;

time_t padStreamTimeoutSecs  = 60;

time_t   padStreamPetTime    = 0;
uint32_t padStreamPetTimeUs  = 0;

uint32_t maxStreamSendDelay1 = 0;
uint32_t maxStreamSendDelay2 = 0;
uint32_t maxStreamSendDelay3 = 0;
uint32_t padStreamPktSent    = 0;
uint32_t padStreamPetted     = 0;
uint32_t padStreamPetDiffMax = 0;
uint32_t padStreamPetDiffMin = -1;
uint32_t padStreamPetDiffAvg = 0;


uint32_t padStreamXid        = 0xfeaf3210;
unsigned int padStreamRand   = 0xfeaf3210;
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
static int            nsamples, nsamples_frag, sampsizld, npkts, channels; /* keep values around (lazyness) */
#define LONGDATA (sampsizld == 2)

static IpBscIf intrf;
static PadStreamStartStopCB start_stop_cb = 0;
static void  *cbarg = 0;

static volatile int isup = 0;

int
padStreamInitialize(void *if_p, PadStreamStartStopCB cb, void *uarg)
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
uint32_t       diff;
	rply->timestampHi = req->timestampHi;
	rply->timestampLo = req->timestampLo;
	rply->xid         = req->xid;
	gettimeofday(&now, 0);

	diff  = (now.tv_sec - padStreamPetTime) * 1000000;
	diff +=  now.tv_usec - padStreamPetTimeUs;

	if ( diff > padStreamPetDiffMax )
		padStreamPetDiffMax = diff;

	if ( diff < padStreamPetDiffMin )
		padStreamPetDiffMin = diff;

	padStreamPetDiffAvg += ((int)(diff - padStreamPetDiffAvg)) >> 8;

	padStreamPetTime   = now.tv_sec;
	padStreamPetTimeUs = now.tv_usec;
	padStreamPetted++;
}

int
padStreamStart(PadRequest req, PadStrmCommand scmd, int me, uint32_t hostip)
{
PadReply        rply = &lpkt_udp_pld(&replyPacket, PadReplyRec);
int             len,chno,msk;
int             rval;
LanIpPacket     pkt;
int             nchannels, nchannels_in_strm;
int             nbytes;

	if ( !intrf )
		return -ENODEV; /* stream has not been initialized yet */

	/* Check elementary parameters */
	channels = (scmd->channels & PADCMD_STRM_CHANNELS_ALL);
	/* courtesy; if existing SW fails to set channels we assume they want ALL */
	if ( 0 == channels || PADCMD_STRM_CHANNELS_ALL == channels ) {
		if ( (scmd->flags & PADCMD_STRM_FLAG_C1) ) {
			return -EINVAL;
		}
		channels  = PADCMD_STRM_CHANNELS_ALL;
		nchannels = PADRPLY_STRM_NCHANNELS;
		chno      = -1; /* keep compiler happy */
	} else {
		/* How many & which channel did they select */
		for ( chno=0, msk = channels;  ! (msk & 1); chno++, msk>>=1 )
			/* nothing else to do */;
		msk >>= 1;
		if ( msk ) {
			/* only 1 or ALL channels supported ATM */
			return -EOPNOTSUPP;
		}
		/* Single-channel must be row-major */
		if ( (scmd->flags & PADCMD_STRM_FLAG_CM) )
			return -EOPNOTSUPP;
		nchannels = 1;
	}
	nchannels_in_strm = nchannels;

	/* Recent lanIpBasic asynchronously updates the ARP cache. If want
	 * to make sure that we have a valid cache entry then we better
	 * do an arpPutEntry() here.
	 * Unfortunately we have to hack-up a pointer to the ethernet
	 * header of the request packet. Uuuugly!
	 * The cleaner method would be issuing a synchronous arp lookup
	 * but we want to avoid sending ARP requests at all cost.
	 */
	pkt  = (LanIpPacket)((void*)req - udpSockUdpBufPayload(0));
	rval = arpPutEntry(intrf, hostip, lpkt_eth(pkt).src, 0); 

	if ( rval )
		return rval;

	LOCK();

		/* cannot change layout and # of samples etc. while up */
		if ( isup ) {
			if  (  (    (scmd->flags & (PADCMD_STRM_FLAG_LE | PADCMD_STRM_FLAG_32))
			         != (rply->strm_cmd_flags & (PADCMD_STRM_FLAG_LE | PADCMD_STRM_FLAG_32)) )
                 || (!! PADRPLY_STRM_IS_CM(rply) != !! (scmd->flags & PADCMD_STRM_FLAG_CM) )
			     || ( nsamples != ntohl(scmd->nsamples) )
			     || ( channels != scmd->channels && (scmd->channels || PADCMD_STRM_CHANNELS_ALL != channels) )
                ) {
			UNLOCK();
			return -EINVAL;
			}
		}

		nsamples_frag = nsamples  = ntohl(scmd->nsamples);
		npkts     = 1;
		sampsizld = (scmd->flags & PADCMD_STRM_FLAG_32) ? 2 : 1;

		nbytes    = (nsamples<<sampsizld) * nchannels;

		padStreamXid = rand_r( &padStreamRand );

		/* common MTU is 1500; subtract ethernet (14), IP (20), UDP (8), padProto (20) = 1438; round
		 * down to next multiple of 16. -> 1424
		 */
		if ( nbytes > 1424 ) {

			/* doesn't fit in one packet */

			/* ATM, if we fragment then all fragments must be 1k in size */
			if ( (nbytes & ((1<<LD_FRAG_SIZE)-1)) ) {
				UNLOCK();
				return -EINVAL;
			}

			/* Furthermore, in row-major layout row-boundaries must be packet boundaries */
			if ( ! (scmd->flags & PADCMD_STRM_FLAG_CM) ) {
				if ( ( (nsamples << sampsizld) & ((1<<LD_FRAG_SIZE)-1) ) ) {
					UNLOCK();
					return -EINVAL;
				}
				/* In row-major format all the data in a packet are
				 * belonging to a single channel.
				 */
				nchannels = 1;
			}

			npkts    = nbytes >> LD_FRAG_SIZE;
			nsamples_frag  = (1<<(LD_FRAG_SIZE - sampsizld)) / nchannels; 
		}

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

			len = ((nsamples_frag*nchannels) << sampsizld) + sizeof(*rply);
			/* Fill in length and IP header checksum etc */
			udpSockHdrsSetlen(&lpkt_udp_hdrs(&replyPacket), len);
			/* Setup Reply */
			rply->version         = req->version;
			rply->type            = PADCMD_STRM | PADCMD_RPLY;
			rply->chnl            = me;
			rply->nBytes          = htons(len);
			rply->stat            = 0;
			rply->strm_cmd_flags  = scmd->flags;
			if ( 1 == nchannels_in_strm ) {
				rply->strm_cmd_flags |= PADCMD_STRM_FLAG_C1;
				PADRPLY_STRM_CHANNEL_SET(rply, chno);
			}
			rply->strm_cmd_idx    = 0;

			isup                  = 1;
		}

		dopet(req, rply);

		rval =  start_stop_cb ? start_stop_cb(req, scmd, cbarg) : 0;

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

static inline int32_t
swapl(int32_t x)
{
uint32_t v = (uint32_t)x;
	v = ((v & 0x00ff00ff) << 8) | ((v>>8) & 0x00ff00ff);
	v = (v >> 16) | (v<<16);
	return (int32_t) v;
}

static void *
streamTest16(void *packetBuffer,
			int idx,
			int nchannels,
			int nsamples,
			int little_endian,
			int column_major,
			void *uarg)
{
int16_t *buf = packetBuffer;
int i,j,k,ch;

	if ( nchannels < PADRPLY_STRM_NCHANNELS ) {
		ch        = nchannels;
		nchannels = 1;
	} else {
		ch        = 0;
	}

	k = idx*nsamples;
	if ( !little_endian != bigEndian() ) {
		if ( column_major ) {
			k *= nchannels;
			for (i=0; i<nsamples*nchannels; i+=nchannels) {
				for (j=0; j<nchannels; j++) {
					buf[i+j] = swap(i+k+j);
				}
			}
		} else {
			for (j=0; j<nchannels; j++, ch=j) {
				for (i=0; i<nsamples; i++) {
					buf[i+j*nsamples] = swap(i+k+(ch<<12));
				}
			}
		}
	} else {
		#define swap(x)	(x)
		if ( column_major ) {
			k *= nchannels;
			for (i=0; i<nsamples*nchannels; i+=nchannels) {
				for (j=0; j<nchannels; j++) {
					buf[i+j] = swap(i+k+j);
				}
			}
		} else {
			for (j=0; j<nchannels; j++, ch=j) {
				for (i=0; i<nsamples; i++) {
					buf[i+j*nsamples] = swap(i+k+(ch<<12));
				}
			}
		}
		#undef swap
	}
	return buf;
}

static void *
streamTest32(void *packetBuffer,
			int idx,
			int nchannels,
			int nsamples,
			int little_endian,
			int column_major,
			void *uarg)
{
int32_t *buf = packetBuffer;
int i,j,k,ch;

	if ( nchannels < PADRPLY_STRM_NCHANNELS ) {
		ch        = nchannels;
		nchannels = 1;
	} else {
		ch        = 0;
	}

	k = idx*nsamples;
	if ( !little_endian != bigEndian() ) {
		if ( column_major ) {
			k *= nchannels;
			for (i=0; i<nsamples*nchannels; i+=nchannels) {
				for ( j=0; j<nchannels; j++ ) {
					buf[i+j] = swapl(i+k+j);
				}
			}
		} else {
			for ( j=0; j<nchannels; j++, ch=j ) {
				for (i=0; i<nsamples; i++) {
					buf[i+j*nsamples] = swapl(i+k+(ch<<16));
				}
			}
		}
	} else {
		#define swapl(x)	(x)
		if ( column_major ) {
			k *= nchannels;
			for (i=0; i<nsamples*nchannels; i+=nchannels) {
				for ( j=0; j<nchannels; j++ ) {
					buf[i+j] = swapl(i+k+j);
				}
			}
		} else {
			for ( j=0; j<nchannels; j++, ch=j ) {
				for (i=0; i<nsamples; i++) {
					buf[i+j*nsamples] = swapl(i+k+(ch<<16));
				}
			}
		}
		#undef swapl
	}
	return buf;
}

static void *
streamTest(void *packetBuffer,
			int idx,
			int channels,
			int nsamples_tot,
			int nsamples_frag,
			int d32,
			int little_endian,
			int column_major,
			void *uarg)
{
	return d32 ? 
        streamTest32(packetBuffer, idx, channels, nsamples_frag, little_endian, column_major, uarg) :
        streamTest16(packetBuffer, idx, channels, nsamples_frag, little_endian, column_major, uarg);
}


static PadStripSimValRec strips;

void
padStreamDumpSimVals(void)
{
	printf("Current simulation values:\n");
	printf("0x%08"PRIx32" 0x%08"PRIx32" 0x%08"PRIx32" 0x%08"PRIx32"\n", strips.val[0], strips.val[1], strips.val[2], strips.val[3]);
	printf("%10"PRIi32" %10"PRIi32" %10"PRIi32" %10"PRIi32"\n", strips.val[0], strips.val[1], strips.val[2], strips.val[3]);
}

PadStreamGetdataProc padStreamSim_getdata = padStreamSim_iir2_getdata;

/* This does not yet support fragmented streams! */
void *
padStreamSim_iir2_getdata(void *packetBuffer,
			int idx,
			int nchannels,
			int nsamples_tot,
			int nsamples_frag,
			int d32,
			int little_endian,
			int column_major,
			void *uarg)
{
int16_t		*buf  = packetBuffer;
int32_t     *bufl = packetBuffer;
int32_t      tmp[PADRPLY_STRM_NCHANNELS];
PadStripSimVal ini  = uarg;
int         swp;
int         i,j,ch;
static unsigned long noise = 1;

	swp    = ( bigEndian() != !little_endian );
	
	if ( nchannels < PADRPLY_STRM_NCHANNELS ) {
		ch = nchannels;
		nchannels = 1;
	} else {
		ch = 0;
	}

	if ( d32 ) {
		if ( swp ) {
			for ( j=0; j<nchannels; j++, ch=j ) {
				tmp[j] = swapl(ini->val[ch]);

			}
		} else {
			for ( j=0; j<nchannels; j++, ch=j ) {
				tmp[j] = ini->val[j];
			}
		}
		if ( column_major ) {
			for ( i=0; i<nsamples_frag; i++ ) {
				for ( j=0; j<nchannels; j++ ) {
					bufl[i + j] = tmp[j];
				}
			}
		} else {
			for ( i=0; i<nsamples_frag; i++ ) {
				for ( j=0; j<nchannels; j++ ) {
					bufl[i + j*nsamples_frag] = tmp[j];
				}
			}
		}
	} else {
		if ( column_major ) {
			for ( j=0; j<nchannels; j++, ch = j ) {
				iir2_bpmsim(buf++, nsamples_frag, ini->val[ch], 0,  &noise, swp, nchannels);
			}
		} else {
			for ( j=0; j<nchannels; j++, ch = j ) {
				iir2_bpmsim(buf, nsamples_frag, ini->val[ch], 0,  &noise, swp, 1);
				buf += nsamples_frag;
			}
		}
	}

	return packetBuffer;
}

#ifdef __mcf5200__
#include "hwtmr.h"

extern uint32_t drvLan9118RxIntBase;

#endif

int
padStreamSend(PadStreamGetdataProc getdata, int type, void *uarg)
{
int            rval = 0;
int            idx;
PadReply       rply = &lpkt_udp_pld(&replyPacket, PadReplyRec);
DrvLan9118_tps plan = lanIpBscIfGetDrv(intrf);
int            len, nchannels, ch;
void          *data_p;
uint32_t       now;
struct timeval now_tv;

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

	now   = (now_tv.tv_sec  - padStreamPetTime) * 1000000;
	now  +=  now_tv.tv_usec - padStreamPetTimeUs;

	if ( now > maxStreamSendDelay3 )
		maxStreamSendDelay3 = now;


	/* just look in the cache - we rely on the RX daemon refreshing it */
	if ( (rval = arpLookup(intrf, lpkt_ip(&replyPacket).dst, lpkt_eth(&replyPacket).dst, 1)) ) {
		UNLOCK();
		return rval;
	}

	if ( (padStreamDebug & 0x1000) ) {
		/* hack timestamp; tag with our us clock */
		rply->timestampLo = htonl(Read_hwtimer());
	}

	rply->strm_cmd_flags &= ~PADRPLY_STRM_FLAG_TYPE_SET(-1);
	rply->strm_cmd_flags |=  PADRPLY_STRM_FLAG_TYPE_SET(type);

	nchannels = (rply->strm_cmd_flags & PADCMD_STRM_FLAG_C1) ? 1 : PADRPLY_STRM_NCHANNELS,

	len = UDPPKTSZ(ntohs(rply->nBytes));

	if ( npkts > 0 ) {
		rply->xid = padStreamXid++;
	}

	for ( idx = 0; idx < npkts; idx++ ) {
		rply->strm_cmd_idx    = idx;
		if ( idx + 1 < npkts )
			rply->strm_cmd_idx |= PADRPLY_STRM_CMD_IDX_MF;

		/* FIXME: Hmm - if we have multiple fragments to send, should
		 * we busy-wait for space?
		 */
		if ( drvLan9118TxPacket(plan, 0, len, 0) ) {
			UNLOCK();
			return -ENOSPC;
		}

		drvLan9118FifoWr(plan, &replyPacket, UDPPKTSZ(sizeof(PadReplyRec)));

		if ( (padStreamDebug & 0x1) ) {
			printf("padStream: idx %i, type %i, nchannels %i, npkts %i\n", idx, type, nchannels, npkts);
		}

		if ( ! (rply->strm_cmd_flags & PADCMD_STRM_FLAG_C1) ) {
			if ( rply->strm_cmd_idx && ! PADRPLY_STRM_IS_CM(rply) ) {
				/* multi-fragment, row-major packet - request only a single channel! */
				ch = idx * PADRPLY_STRM_NCHANNELS / npkts;
				nchannels = 1;
			} else {
				ch = PADRPLY_STRM_NCHANNELS;
			}
		} else {
			ch = PADRPLY_STRM_CHANNEL_GET( rply );
		}

		if ( (data_p=getdata(
						&rply->data,
						idx,
						ch,
						nsamples,
						nsamples_frag, 
						rply->strm_cmd_flags & PADCMD_STRM_FLAG_32,
						rply->strm_cmd_flags & PADCMD_STRM_FLAG_LE,
						PADRPLY_STRM_IS_CM(rply),
						uarg)) ) {
			drvLan9118FifoWr(plan, data_p, ((nsamples_frag*nchannels) << sampsizld));
		}
		/* else ['getdata' returned NULL] the getdata method already
		 * wrote to the TX FIFO
		 */

		drvLan9118TxUnlock(plan);
	}

	UNLOCK();

	padStreamPktSent++;

#ifdef __mcf5200__
	now = Read_hwtimer() - drvLan9118RxIntBase;
	if ( now > maxStreamSendDelay2 )
		maxStreamSendDelay2 = now;
#endif

	return 0;
}

int
padStreamTest(int type)
{
	return padStreamSend(streamTest, type, 0);
}

int
padStreamSim(PadSimCommand scmd, uint32_t hostip)
{
int dosend = 1;
int err    = 0;

	if ( !intrf )
		return -ENODEV; /* stream has not been initialized yet */

	if ( scmd ) {
		dosend =  ! (PADCMD_SIM_FLAG_NOSEND & scmd->flags );

		LOCK();
			if ( dosend ) {
				if ( ! peer.ip ) {
					err = -ENOTCONN;
				} else if ( hostip && peer.ip != hostip ) {
					err = -EADDRINUSE;
				}
			}
			if ( !err ) {
				strips.val[0] = ntohl(scmd->a);
				strips.val[1] = ntohl(scmd->b);
				strips.val[2] = ntohl(scmd->c);
				strips.val[3] = ntohl(scmd->d);
			}
		UNLOCK();

		if ( err )
			return err;

	}

	return  dosend ? padStreamSend(padStreamSim_getdata, 0, &strips) : 0;
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
					rval = start_stop_cb(0, 0, cbarg);

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
