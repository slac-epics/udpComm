#include <udpComm.h>
#include "padProto.h"
#include <padStream.h>

#include <errno.h>

#include <epicsInterrupt.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <epicsVersion.h>
#include <registryFunction.h>
#include <errlog.h>
#include <drvSup.h>
#include <iocsh.h>
#include <registry.h>
#include <cantProceed.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <assert.h>

#include <epicsString.h>

#undef __STRICT_ANSI__
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include <aiRecord.h>
#include <subRecord.h>
#include <alarm.h>
#include <recGbl.h>
#include <osiSock.h>

#include <drvPadUdpComm.h>
#include <drvPadUdpCommIO.h>
#include <wavBuf.h>

#include <menuPriority.h>
#include <fidProcess.h>

/* Up to 3.14.9 an event-time provider could override the
 * meaning of event 0 -- as of 3.14.10 event 0 is 
 * caught + handled early, i.e., before the EVR code can
 * do it's tricks...
 */
#if !defined(EPICS_VERSION) || !defined(EPICS_REVISION) || !defined(EPICS_MODIFICATION)
#error "Unknown EPICS base version"
#elif    EPICS_VERSION > 3 \
    || 3 == EPICS_VERSION && EPICS_REVISION > 14 \
	|| 3 == EPICS_VERSION && 14 == EPICS_REVISION && EPICS_MODIFICATION > 9

#ifdef TEST_ONLY
#define TS1_OR_4_TIME   epicsTimeEventCurrentTime
#else
#define TS1_OR_4_TIME   (epicsTimeEventBestTime)
#define ENSURE_EVR_INITED
#include <evrTime.h>
#endif

#else
/* This has the nice side-effect that we get wall-clock time if there is
 * no EVR (testing).
 */
#define TS1_OR_4_TIME	0
#endif

#define IIR128(old, new) ((old)*(1.0-1.0/128.) + (new)/128.)

#ifndef PI
#define PI   3.1415926535897932385
#endif

#define FRAG_MAX ((uintptr_t)(-1))

#define RAD ((PI)/180.0)

#ifdef __PPC__
uint32_t	roundtripMax = 0;
uint32_t	roundtripAvg = 0;
#endif


static epicsEventId     shutdownEvent;
static int              workLoop = 1;

int			drvPadUdpCommDebug = 0;

int         drvPadUdpCommRetryQuery = 3;

volatile DrvPadUdpCommHWTime drvPadUdpCommMaxCook = 0;

#if MAX_BPM > 31
#error "too many channels -- don't fit in 32-bit mask"
#endif
volatile uint32_t bpmUpToDate[PADRPLY_STRM_NUM_KINDS]    = { 0, 0, 0};
volatile uint32_t drvPadUdpCommChannelsInUseMask = 0;

static char simMode[MAX_BPM] = {0};

/* Simulated BPMs                      */
volatile struct {
	PadRequestRec		req;
	PadSimCommandRec	simbpm[MAX_BPM];
} padSimulateCmd;

struct {
	PadRequestRec		req;
	PadStrmCommandRec	cmd[MAX_BPM];
} padStrmCmd;

/* Cache # of samples per ADC */
static unsigned         padNSamples[MAX_BPM];


static epicsMutexId padStrmCmdLock;

DrvPadUdpCommCallbacksRec drvPadUdpCommCallbacks = { 0 };

DrvPadUdpCommPrefsRec     drvPadUdpCommPrefs     = { 0 };

volatile DrvPadUdpCommHWTime drvPadUdpCommHWTimeBaseline [MAX_BPM] = { 0 };
volatile DrvPadUdpCommHWTime drvPadUdpCommPktTimeBaseline          = { 0 };

static int udpCommSetup(const char *arg, uint32_t *peer_ip_p, int *peer_port_p);

static UdpCommPkt drvPadUdpCommAllocPacket(int fd)
{
	return udpCommAllocPacket();
}

static DrvPadUdpCommIORec io = {
	setup:    udpCommSetup,
	open:     udpCommSocket,
	close:    udpCommClose,
	connect:  udpCommConnect,
	send:     udpCommSend,
	recv:     udpCommRecv,
	bufptr:   udpCommBufPtr,
	alloc:    drvPadUdpCommAllocPacket,
	creatref: udpCommRefPacket,
	free:     udpCommFreePacket,
	padIoReq: padRequest,
};

/* This is actually optimized away by gcc */
static int
isbe()
{
union {
	uint8_t  b[2];
	uint16_t test;
} endian = { b: {0xbe, 00 } };
	return endian.test == 0xbe00;
}

static int
drvPadUdpCommGetNsamplesDefault()
{
	return 128;
}

static void
strmCmdInit(int d32, int col_maj)
{
int i;

	padStrmCmd.req.version = PADPROTO_VERSION4;
	padStrmCmd.req.nCmds   = MAX_BPM;
	padStrmCmd.req.cmdSize = sizeof(padStrmCmd.cmd[0]);
	for ( i=0; i<MAX_BPM; i++ ) {
		padStrmCmd.cmd[i].type      = PADCMD_NOP | PADCMD_QUIET;
		padStrmCmd.cmd[i].flags     = (!isbe()) ? PADCMD_STRM_FLAG_LE : 0;
		/* VME driver only supports column-major layout */
		if ( col_maj )
			padStrmCmd.cmd[i].flags |= PADCMD_STRM_FLAG_CM;
		if ( d32 )
			padStrmCmd.cmd[i].flags |= PADCMD_STRM_FLAG_32;
		padStrmCmd.cmd[i].port      = htons(drvPadUdpCommPort + 1);
		padStrmCmd.cmd[i].nsamples  = htonl(drvPadUdpCommPrefs.nsamples);
		padStrmCmd.cmd[i].channels  = PADCMD_STRM_CHANNELS_ALL;
		padNSamples[i]              = drvPadUdpCommPrefs.nsamples;
	}
	padStrmCmdLock = epicsMutexMustCreate();
}

int
drvPadUdpCommStrmStopReq(int channel)
{
int            key, rval;
epicsTimeStamp ts;

	if ( channel < 0 || channel >= MAX_BPM )
		return -1;

	key = epicsInterruptLock();
		drvPadUdpCommChannelsInUseMask &= ~(1<<channel);
	epicsInterruptUnlock(key);

	epicsMutexLock(padStrmCmdLock);

		padStrmCmd.cmd[channel].type = PADCMD_NOP | PADCMD_QUIET;

	epicsMutexUnlock(padStrmCmdLock);

	if ( epicsTimeGetEvent(&ts, TS1_OR_4_TIME) )
		epicsTimeGetCurrent(&ts);
	
	rval = io.padIoReq(
				drvPadUdpCommSd,
				channel,
				PADCMD_STOP | PADCMD_QUIET,
				drvPadUdpCommGetXid(),
				ts.secPastEpoch, ts.nsec,
				0,
				0,
				drvPadUdpCommTimeout);

	if ( rval ) {
		epicsPrintf("drvPadUdpCommStrmStopReq: Unable to stop stream: %s\n", strerror(-rval));
	}

	return rval;
}

int
drvPadUdpCommStrmStartReq(int channel)
{
int               key, rval;
PadStrmCommandRec scmd;
epicsTimeStamp    ts;

	if ( channel < 0 || channel >= MAX_BPM )
		return -1;

	epicsMutexLock(padStrmCmdLock);

		padStrmCmd.cmd[channel].type = PADCMD_STRM | PADCMD_QUIET;
		scmd = padStrmCmd.cmd[channel];
	
	epicsMutexUnlock(padStrmCmdLock);

	if ( epicsTimeGetEvent(&ts, TS1_OR_4_TIME) )
		epicsTimeGetCurrent(&ts);
	
		rval = io.padIoReq(
					drvPadUdpCommSd,
					channel,
					PADCMD_STRM | PADCMD_QUIET,
					drvPadUdpCommGetXid(),
					ts.secPastEpoch, ts.nsec,
					&scmd,
					0,
					drvPadUdpCommTimeout);


	key = epicsInterruptLock();
		drvPadUdpCommChannelsInUseMask |= (1<<channel);
	epicsInterruptUnlock(key);

	if ( rval ) {
		epicsPrintf("drvPadUdpCommStrmStartReq: Unable to start stream: %s\n", strerror(-rval));
	}

	return rval;
}

int
drvPadUdpCommStrmSetChannels(int padChannel, int adcChannels)
{
int rval;

	if ( (padChannel < 0 && adcChannels < 0) || padChannel >= MAX_BPM ) {
		/* invalid argument */
		return -1;
	}

	if ( adcChannels < 0 ) {
		epicsMutexLock(padStrmCmdLock);
			rval = padStrmCmd.cmd[padChannel].channels;
		epicsMutexUnlock(padStrmCmdLock);
		return rval;
	}

	if ( ! drvPadUdpCommPrefs.nchannels_dynamic ) {
		epicsPrintf("drvPadUdpCommStrmSetChannels: client-driver forbids changing sample number\n");
		return -1;
	}

	if ( adcChannels > PADCMD_STRM_CHANNELS_ALL || adcChannels <= 0 ) {
		epicsPrintf("drvPadUdpCommStrmSetChannels: unsupported channel mask: 0x%x\n", adcChannels);
		return -1;
	}

	epicsMutexLock(padStrmCmdLock);
	if ( padChannel < 0 ) {
		if ( (drvPadUdpCommChannelsInUseMask & ((1<<MAX_BPM)-1)) ) {
			epicsMutexUnlock(padStrmCmdLock);
			epicsPrintf("drvPadUdpCommSetChannels: cannot change ADC channels while any padChannel is started/streaming\n");
			return -1;
		}
		for ( padChannel = 0; padChannel < MAX_BPM; padChannel++ ) {
			padStrmCmd.cmd[padChannel].channels = adcChannels;
		}
		rval = 0;
	} else {
		if ( (drvPadUdpCommChannelsInUseMask & (1<<padChannel)) ) {
			epicsMutexUnlock(padStrmCmdLock);
			epicsPrintf("drvPadUdpCommSetChannels: cannot change ADC channels while padChannel is started/streaming\n");
			return -1;
		}
		rval = padStrmCmd.cmd[padChannel].channels;
		padStrmCmd.cmd[padChannel].channels = adcChannels;
	}
	epicsMutexUnlock(padStrmCmdLock);

	return rval;
}

int
drvPadUdpCommStrmSetNSamples(int channel, int nsamples)
{
int rval;
int nsamples_rev;

#if 0
	if ( nsamples > 170 ) {
		/* wouldn't fit into std. ethernet packet */
		return -1;
	}
#endif

	if ( (channel < 0 && nsamples <= 0) || channel >= MAX_BPM )
		return -1;

	if ( nsamples <= 0 ) {
		return padNSamples[channel];
	}

	if ( ! drvPadUdpCommPrefs.nsamples_dynamic ) {
		epicsPrintf("drvPadUdpCommStrmSetNSamples: client-driver forbids changing sample number\n");
		return -1;
	}

	nsamples_rev = htonl(nsamples);

	epicsMutexLock(padStrmCmdLock);
	if ( channel < 0 ) {
		if ( (drvPadUdpCommChannelsInUseMask & ((1<<MAX_BPM)-1)) ) {
			epicsMutexUnlock(padStrmCmdLock);
			epicsPrintf("drvPadUdpCommSetNSamples: cannot change #-of samples while any channel is started/streaming\n");
			return -1;
		}
		for ( channel = 0; channel < MAX_BPM; channel++ ) {
			padStrmCmd.cmd[channel].nsamples = nsamples_rev;
			padNSamples[channel]             = nsamples;
		}
		rval = 0;
	} else {
		if ( (drvPadUdpCommChannelsInUseMask & (1<<channel)) ) {
			epicsMutexUnlock(padStrmCmdLock);
			epicsPrintf("drvPadUdpCommSetNSamples: cannot change #-of samples while channel is started/streaming\n");
			return -1;
		}
		rval = padNSamples[channel];
		padStrmCmd.cmd[channel].nsamples = nsamples_rev;
		padNSamples[channel]             = nsamples;
	}
	epicsMutexUnlock(padStrmCmdLock);

	return rval;
}

#define DIAG_SCAN_LIMIT 1 /* second; don't scan diagnostic records faster than this */

/* 'normal' scan list; processed on every beam pulse */
static IOSCANPVT   scanlist[MAX_BPM];
/*
 * 'diagnostics' scan list; processed on every beam pulse
 * but not faster than ~1Hz; i.e., at a decimated rate.
 * In particular, all diagnostic waveforms should be on
 * this list.
 *
 * Which list they end up on is determined by a record's
 * PRIO field.
 */
static IOSCANPVT   dScanlist[MAX_BPM];
static epicsUInt32 lastTimeDiagScanned[MAX_BPM] = {0,};

int drvPadUdpCommSd       = -1;
int drvPadUdpCommPort     = PADPROTO_PORT;
int drvPadUdpCommTimeout  = 5000;
double drvPadUdpCommStrmTimeout = 5.0;
uint32_t drvPadUdpCommPeerAddr = INADDR_NONE;
int drvPadUdpCommHasSetup = 0;

WavBuf wbStaged[WAV_BUF_NUM_SLOTS][WAV_BUF_NUM_KINDS] = {{0}};

PadReply
padReplyFind(void *data_p)
{
	return data_p ? (PadReply)io.bufptr(data_p) : 0;
}

int
padDataFree(WavBuf wb)
{
int  i;
void *p;
	for ( i=0; i<wb->segs.nsegs; i++ ) {
		if ( wb->segs.data[i] ) {
			p = (void*)((uintptr_t)wb->segs.data[i] - (uintptr_t)((PadReply)io.bufptr(0))->data);
			io.free( p );
		}
	}
	return 0;
}

void
padReplyFree(PadReply reply)
{
	if ( reply ) {
		io.free( (void*)( (uintptr_t)reply - (uintptr_t)io.bufptr(0) ) );
	}
}

void
padReplyRef(PadReply reply)
{
	io.creatref( (void*)( (uintptr_t)reply - (uintptr_t)io.bufptr(0) ) );
}

static long 
padUdpCommReport(int level)
{
	epicsPrintf("Udp Communication with PAD\n");
	if ( level > 0 ) {
		epicsPrintf("Channels in use mask  0x%08"PRIx32"\n", drvPadUdpCommChannelsInUseMask);
		epicsPrintf("Max Cook time (ticks) %d\n", (unsigned)drvPadUdpCommMaxCook);
	}
	return 0;
}

uint32_t
drvPadUdpCommGetXid()
{
uint32_t rval;
	/* using timebase can be useful as a baseline for timing purposes */
#if defined(__PPC__) 
#if defined(__rtems__)
	/* reading timebase via SPR also works on E500 CPUs */
	__asm__ volatile("mfspr %0, 268":"=r"(rval));
#else
	/* mfspr requires privileged mode */
	__asm__ volatile("mftb  %0":"=r"(rval));
#endif
#elif defined(__i386__)
	uint32_t hi;
	__asm__ volatile("rdtsc" : "=a"(rval), "=d"(hi));
#else
static volatile uint32_t xid = 0;
int      key;

	key = epicsInterruptLock();
	rval = xid++;
	epicsInterruptUnlock(key);
#endif
	return rval;
}

int
drvPadUdpCommSendStartCmd(int nsamples, uint32_t xid, int channel)
{
PadStrmCommandRec scmd;
epicsTimeStamp    ts;

	scmd.type     = PADCMD_STRM;
	scmd.flags    = (!isbe()) ? PADCMD_STRM_FLAG_LE : 0;
	if ( drvPadUdpCommPrefs.d32 )
		scmd.flags |= PADCMD_STRM_FLAG_32;
	scmd.port     = htons(drvPadUdpCommPort + 1);
	scmd.nsamples = htonl(nsamples);

	/* Dont ask for a reply if we broadcast */
	if ( channel < 0 )
		scmd.type |= PADCMD_QUIET;

	if ( epicsTimeGetEvent(&ts, TS1_OR_4_TIME) )
		epicsTimeGetCurrent(&ts);
	
	drvPadUdpCommPktTimeBaseline = drvPadUdpCommHWTime();

	return io.padIoReq(
				drvPadUdpCommSd,
				channel,
				scmd.type,
				xid,
				ts.secPastEpoch, ts.nsec,
				&scmd,
				0,
				drvPadUdpCommTimeout);
}

int
drvPadUdpCommSendStartAll(epicsTimeStamp *ts_p)
{
int            rval;
epicsTimeStamp ts;

	epicsMutexLock(padStrmCmdLock);

	padStrmCmd.req.xid = htonl(drvPadUdpCommGetXid());

	if ( !ts_p ) {
		if ( epicsTimeGetEvent(&ts, TS1_OR_4_TIME) )
			epicsTimeGetCurrent(&ts);
		ts_p = &ts;
	}

	padStrmCmd.req.timestampHi = htonl(ts_p->secPastEpoch);
	padStrmCmd.req.timestampLo = htonl(ts_p->nsec);
	
	rval = io.send(drvPadUdpCommSd, &padStrmCmd, sizeof(padStrmCmd));

	epicsMutexUnlock(padStrmCmdLock);

	return rval;
}

static int
drvPadUdpCommPostRaw(UdpCommPkt pkt, PadDataKind kind)
{
WavBuf   pb = 0;
PadReply rply;
unsigned chan;
unsigned idx;
int      rval = -1;
uint32_t oxid;

	if ( !pkt )
		return rval;

	rply     = padReplyFind(pkt);

	chan     = rply->chnl;

	if ( chan >= WAV_BUF_NUM_SLOTS || (unsigned)kind >= WAV_BUF_NUM_KINDS ) {
		epicsPrintf("drvPadUdpCommPostRaw: invalid chan/kind (%u/%u), dropping...\n", chan, kind);
		goto bail;
	}

	/* FIXME: If we have multiple listener threads then wbStaged should be protected */
	if ( (pb = wbStaged[chan][kind]) ) {
		oxid = ((PadReply)((uintptr_t)pb->segs.data[pb->segs.nsegs-1] - (uintptr_t)((PadReply)0)->data))->xid;
		if ( rply->xid != oxid ) {
			/* fragment already part of a new stream packet */
			wavBufFree( pb );
			pb = wbStaged[chan][kind] = 0;

			if ( ( drvPadUdpCommDebug & 2 ) ) {
				errlogPrintf("postRaw[%i][%i]: XID overrun, dropping (old: 0x%08"PRIx32", new: 0x%08"PRIx32"\n", chan, kind, rply->xid, oxid);
			}
		} else {
			if ( pb->type != ((rply->strm_cmd_flags & PADCMD_STRM_FLAG_32) ? WavBufInt32 : WavBufInt16) ) {
				epicsPrintf("drvPadUdpCommPostRaw: inconsistent type (%u), dropping\n", pb->type);
				goto bail;
			}
			if ( pb->m    != PADRPLY_STRM_CHANNELS_IN_STRM(rply) ) {
				epicsPrintf("drvPadUdpCommPostRaw: inconsistent m-rows, dropping\n");
				goto bail;
			}
			if ( pb->segs.n != PADRPLY_STRM_NSAMPLES( rply ) / pb->segs.m ) {
				epicsPrintf("drvPadUdpCommPostRaw: inconsistent segment size, dropping\n");
				goto bail;
			}
		}
	}

	if ( ! pb ) {
		if ( !(pb = wavBufAlloc()) ) {
			/* Just drop */
			epicsPrintf("drvPadUdpCommPostRaw: unable to get waveform buf header -- configure more!!\n");
			goto bail;
		}
		pb->free = padDataFree; 
		if ( PADRPLY_STRM_IS_CM(rply) ) {
			pb->flags = WavBufFlagCM;
		} else {
			pb->flags = 0;
		}
		pb->type    = (rply->strm_cmd_flags & PADCMD_STRM_FLAG_32) ? WavBufInt32 : WavBufInt16;
		pb->m       = PADRPLY_STRM_CHANNELS_IN_STRM( rply );
		pb->segs.m  = PADRPLY_STRM_CHANNELS_IN_FRAG( rply );
		pb->segs.n  = PADRPLY_STRM_NSAMPLES( rply ) / pb->segs.m;
		/* pb->n is only known once we have the last fragment */
		pb->usrData = (void*)FRAG_MAX;
		pb->ts.secPastEpoch  = ntohl(rply->timestampHi);
		pb->ts.nsec          = ntohl(rply->timestampLo);

		if ( ( drvPadUdpCommDebug & 2 ) ) {
			errlogPrintf("postRaw[%i][%i]: new buffer: m %u, segs.m %u, segs.n %u\n", chan, kind, pb->m, pb->segs.m, pb->segs.n);
		}
	}

	idx = PADRPLY_STRM_CMD_IDX_GET( rply->strm_cmd_idx );

	if ( idx >= WAV_BUF_NUM_SEGS ) {
		epicsPrintf("drvPadUdpCommPostRaw: number of segments too big; dropping\n");
		goto bail;
	}

	if ( ( drvPadUdpCommDebug & 2 ) ) {
		errlogPrintf("postRaw[%i][%i]: idx %i, usrData: %"PRIiPTR"\n", chan, kind, idx, (intptr_t)pb->usrData);
	}

	if ( idx + 1 > pb->segs.nsegs )
		pb->segs.nsegs = idx + 1;
	pb->segs.data[idx] = rply->data;
	pkt                = 0;

	if ( ( drvPadUdpCommDebug & 2 ) ) {
		errlogPrintf("postRaw[%i][%i]: idx %i\n", chan, kind, idx);
	}

	if ( ! (PADRPLY_STRM_CMD_IDX_MF & rply->strm_cmd_idx) ) {
		/* found the last fragment */
		pb->usrData = (void*)( (uintptr_t)pb->usrData - FRAG_MAX + idx + 1 - 1 );
		pb->n       = (idx + 1) * PADRPLY_STRM_NSAMPLES(rply) / pb->m;
	} else {
		pb->usrData = (void*)( (uintptr_t)pb->usrData - 1 );
	}

	if ( ( drvPadUdpCommDebug & 2 ) ) {
		errlogPrintf("postRaw[%i][%i]: new usrData: %"PRIiPTR"\n", chan, kind, (intptr_t)pb->usrData);
	}

	if ( 0 == (uintptr_t)pb->usrData ) {
		/* All fragments assembled */

		/* send buffer for waveforms to pick up */
		if ( 0 == wavBufPost(chan, kind, pb) ) {
			rval = 0;
		}
		pb = 0;
	}
	wbStaged[chan][kind] = pb;
	pb                   = 0;

bail:
	if ( pb ) {
		wavBufFree( pb );
		wbStaged[chan][kind] = 0;
	}
	if ( pkt )
		io.free( pkt );
	return rval;
}

int
drvPadUdpCommPostReply(PadReply rply, PadDataKind kind)
{
	return drvPadUdpCommPostRaw( (UdpCommPkt)( (uintptr_t)rply - (uintptr_t)io.bufptr(0) ) , kind);
}

static void
drvPadUdpCommListener(void *arg)
{
int         sd;
UdpCommPkt  pkt = 0;
PadReply    rply;
PadDataKind kind, new_kind;
unsigned    chan;
unsigned    nsamples;
int         layout_cm;
int         cook_stat;
int         key;
int         posted;
epicsTimeStamp         now;
/* Keep a copy of callbacksRec which is task-specific. If the 'init' callback
 * modifies/sets 'cookClosure' then that field is maintained per task.
 * We do this because we eventually would like to support multiple listeners
 * on multi-core CPUs.
 */
DrvPadUdpCommCallbacksRec cb = drvPadUdpCommCallbacks;
DrvPadUdpCommHWTime    tDiff;
int         bad_version_count = 0;
unsigned int orig_prio;

	/* Lower priority during initialization */

	orig_prio = epicsThreadGetPrioritySelf();

	epicsThreadSetPriority( epicsThreadGetIdSelf(), epicsThreadPriorityLow );

	if ( cb.init )
		cb.init(&cb);

	/* Open a socket for incoming streamed data */

	sd = io.open(drvPadUdpCommPort + 1);

	if ( sd < 0 ) {
		epicsPrintf("drvPadUdpCommListener -- unable to create socket: %s\n",
			strerror(-sd));
		return;
	}

	epicsThreadSetPriority( epicsThreadGetIdSelf(), orig_prio );

	while ( workLoop ) {
		/* release old buffer */
		io.free(pkt);

		/* keep sending start command until we receive something */
		pkt = io.recv(sd, -1);

		if ( 0 == pkt ) {
			cantProceed("FATAL ERROR: io.recv(timeout_forever) returned NULL\n");
		}

		tDiff    = drvPadUdpCommHWTime();

		rply     = padReplyFind(pkt);

		if ( PADPROTO_VERSION4 != rply->version ) {
			bad_version_count++;
			if ( (bad_version_count & 0xff) == 1 ) {
				errlogPrintf("drvPadUdpCommListener: dropped %u unsupported version %u packets (need %u)\n", bad_version_count, rply->version, PADPROTO_VERSION4);
			}
			continue;	
		}

#ifdef __PPC__
		{
			uint32_t tbl = drvPadUdpCommHWTime();
			tbl -= ntohl(rply->timestampLo);
			if ( tbl > roundtripMax )
				roundtripMax = tbl;
			/* to read AVG, divide by 16 */
			roundtripAvg += (tbl - (roundtripAvg >> 4));
		}
#endif

		chan     = rply->chnl;
		/* Don't bother to take the padStrmCmdLock -- the
		 * stream must be stopped for [chan] while the number
		 * can be modified.
		 */
		nsamples = padNSamples[chan];

		if ( chan >= MAX_BPM ) {
			/* drop invalid channels */
			errlogPrintf("drvPadUdpCommListener: invalid BPM channel #%i\n", chan);
	continue;
		}

		drvPadUdpCommHWTimeBaseline[chan]  = tDiff;

		/* Layout (column-major or row-major) */
		layout_cm = PADRPLY_STRM_IS_CM(rply);

		if ( isbe() != !(rply->strm_cmd_flags & PADCMD_STRM_FLAG_LE) ) {
			errlogPrintf("drvPadUdpCommListener: bad data endianness\n");
	continue;
		}

		kind = PADRPLY_STRM_FLAG_TYPE_GET(rply->strm_cmd_flags);

#ifdef TEST_ONLY
		/* perform fiducial-time processing [assuming there is only
		 * a single PAD in simulation & TEST_ONLY mode].
		 * We don't have a timing
		 * system to synchronously execute stuff at 'fiducial-time'
		 * so we just do it prior to handling the one-and-only PAD
		 */
		if ( PadDataBpm == kind ) {
			fidTestProcess();
		}
#endif

		if ( drvPadUdpCommDebug & 1 ) {
			errlogPrintf("STRM: C %2u IDX %3u %s N %3u K %1u TS: 0x%08"PRIx32" XID: 0x%08"PRIx32"\n",
				chan,
				PADRPLY_STRM_CMD_IDX_GET(rply->strm_cmd_idx),
				(PADRPLY_STRM_CMD_IDX_MF & rply->strm_cmd_idx) ? "(MF)":"    ",
				nsamples,
				kind,
				ntohl(rply->timestampLo),
				ntohl(rply->xid));
		}

		if ( cb.cook ) {
			cook_stat = cb.cook(rply, nsamples, layout_cm, kind, cb.cookClosure);
		} else {
			cook_stat = -1;
		}

		if ( drvPadUdpCommDebug & 1 ) {
			errlogPrintf("STRM: cook_stat %i\n", cook_stat);
		}

		posted = 0;

		if ( cook_stat >= 0 ) {
			/* Pet up-to-date status */
			key = epicsInterruptLock();
				bpmUpToDate[kind] |= (1<<chan);
			epicsInterruptUnlock(key);

			new_kind = cook_stat & ~PAD_UDPCOMM_COOK_STAT_DEBUG_MASK;

			switch ( (PAD_UDPCOMM_COOK_STAT_DEBUG_MASK & cook_stat) ) {
				case PAD_UDPCOMM_COOK_STAT_DEBUG_FLG_DOSCAN:
					cook_stat = 0;
					/* fall thru */
				case PAD_UDPCOMM_COOK_STAT_DEBUG_FLG_NOSCAN:
					io.creatref( pkt );
					if (0 == drvPadUdpCommPostRaw( pkt, new_kind )) {
						posted++;
					}
					/* fall thru */
				default:
				break;
			}
		}

		if ( cook_stat )
			continue;

		/* If pkt is NULL this doesn't do anything */
		if ( 0 == drvPadUdpCommPostRaw( pkt, kind ) )
			posted++;
		pkt = 0;

		/* Scan lists only on the beam pulse. Waveforms
		 * listening to the cal pulses will also be on
		 * the same scanlist.
		 */
		if ( PadDataBpm == kind && posted ) {
			scanIoRequest(scanlist[chan]);
			epicsTimeGetCurrent(&now);
			if ( now.secPastEpoch - lastTimeDiagScanned[chan] >= DIAG_SCAN_LIMIT ) {
				lastTimeDiagScanned[chan] = now.secPastEpoch;
				scanIoRequest(dScanlist[chan]);
			}
		}
		tDiff   = drvPadUdpCommHWTime() - fidTimeBaseline;

		if ( tDiff > drvPadUdpCommMaxCook )
			drvPadUdpCommMaxCook = tDiff;
	}

        epicsEventSignal(shutdownEvent);

}

static void
drvPadUdpCommWatchdog(void *arg)
{
int i, err = 0;

DrvPadUdpCommCallbacks cb = &drvPadUdpCommCallbacks;

	while ( workLoop ) {
		/* Invalidate 'up-to-date flags' */
		int			key,j;
		uint32_t    timedout[PADRPLY_STRM_NUM_KINDS], t, s, sa;

		/*  s: bitfield of all channels that are in use and are timed out
         *     for at least one kind of data.
		 * sa: bitfield of all channels that are in use and are timed out
         *     for all kinds of data.
         */
		s   = 0;
		sa  = 0xffffffff;
		key = epicsInterruptLock();
			for ( j=0; j<PADRPLY_STRM_NUM_KINDS; j++ ) {
				if ( (cb->watchKinds & (1<<j)) ) {
					timedout[j] = drvPadUdpCommChannelsInUseMask & ~bpmUpToDate[j];
					s  |= timedout[j];
					sa &= timedout[j];
					bpmUpToDate[j] = 0;
				}
			}
		epicsInterruptUnlock(key);

		/* If we get nothing at all start all streams (maybe they rebooted the PAD) */
		if ( sa == drvPadUdpCommChannelsInUseMask ) {
			drvPadUdpCommSendStartAll(0);
		}

		if ( cb->watchdog ) {
			/* Only bark if all kinds of data are timed-out */
			for ( i=0, t = sa; t; t>>=1, i++ ) {
				if ( (t & 1) ) {
					for ( j=0; j<PADRPLY_STRM_NUM_KINDS; j++ )
						cb->watchdog(i,j,cb->watchdogClosure);
				}
			}
		}

		/* Scan the channels that had any kind of timeout */
		for ( i=0, t=s; t; t>>=1, i++ ) {
			if ( (t & 1) ) {
				scanIoRequest(scanlist[i]);
				scanIoRequest(dScanlist[i]);
			}
		}

		if ( err < 0 ) {
			errlogPrintf("drvPadUdpCommSimulator: sending simulator data failed\n");
			epicsThreadSleep(10.0);
		}
		epicsThreadSleep(drvPadUdpCommStrmTimeout);
	}
}

int
drvPadUdpCommGetSimMode(unsigned chnl)
{
	return chnl >= MAX_BPM ? -1 : simMode[chnl];
}

int 
drvPadUdpCommSimulatePos(unsigned pad, int mode, float x, float y, float q, float phi, float psi, void *parms)
{
int32_t   vals[PADRPLY_STRM_NCHANNELS];

int       key;

	if ( q < 0.0 )
		q = 0.0;

	if ( pad >= MAX_BPM )
		return -1;

	if ( !drvPadUdpCommCallbacks.sim_pos )
		return -1;

	simMode[pad] = mode; 

	if ( mode ) {

		if ( drvPadUdpCommCallbacks.sim_pos(vals, x, y, q, phi, psi, parms) )
			return -1;

		key = epicsInterruptLock();
			padSimulateCmd.simbpm[pad].a    = htonl(vals[0]);
			padSimulateCmd.simbpm[pad].b    = htonl(vals[1]);
			padSimulateCmd.simbpm[pad].c    = htonl(vals[2]);
			padSimulateCmd.simbpm[pad].d    = htonl(vals[3]);
			padSimulateCmd.simbpm[pad].type = PADCMD_SIM | PADCMD_QUIET;
			/* can only get here if sim-mode is enabled */
			if ( SIM_HW_TRIGGERED == mode ) {
				padSimulateCmd.simbpm[pad].flags |=  PADCMD_SIM_FLAG_NOSEND;
			} else {
				padSimulateCmd.simbpm[pad].flags &= ~PADCMD_SIM_FLAG_NOSEND;
			}
		epicsInterruptUnlock(key);
	} else {
		key = epicsInterruptLock();
		/* Switch simulate mode off */
		padSimulateCmd.simbpm[pad].type     = PADCMD_NOP | PADCMD_QUIET;
		epicsInterruptUnlock(key);
	}

	if ( SIM_OFF != mode )
		io.send( drvPadUdpCommSd, (void*)&padSimulateCmd.req, sizeof(padSimulateCmd));

	return 0;
}


static void
simInit()
{
int i;
	/* Initialize the simulator command */
	padSimulateCmd.req.version = PADPROTO_VERSION4;
	padSimulateCmd.req.nCmds   = MAX_BPM;
	padSimulateCmd.req.cmdSize = sizeof(padSimulateCmd.simbpm[0]);

	for ( i=0; i<MAX_BPM; i++ ) {
		padSimulateCmd.simbpm[i].type = PADCMD_NOP | PADCMD_QUIET;
		padSimulateCmd.simbpm[i].a =
		padSimulateCmd.simbpm[i].b =
		padSimulateCmd.simbpm[i].c =
		padSimulateCmd.simbpm[i].d = htonl(0);
	}
}

static void
set_pref(unsigned flag, unsigned sup_on, unsigned sup_off, int *pref_p, const char *msg)
{
	switch ( ((flag & sup_on) ? 2:0)
            |((flag & sup_off) ? 1:0) ) {
		case 0: /* nothing supported -- should never get here */
			epicsPrintf("ERROR: drvPadUdpComm -- digitizer seems to support option '%s' neither on nor off\n", msg);
			*pref_p = -1;
			break;
		case 1: /* only OFF supported */
			*pref_p = 0;	
			break;
		case 2: /* only ON  supported */
			*pref_p = 1;
			break;
		case 3: /* both supported */
			*pref_p = -1;
			break;
	}
}

static int
check_pref(int *proposed, int *requested, int dflt, const char *msg)
{
	if ( *requested < 0 && (*requested = *proposed) < 0 ) {
		*requested = dflt;
		return 0;
	}

	if ( *proposed < 0 || *proposed == *requested )
		return 0;

	epicsPrintf("drvPadUdpComm -- FATAL ERROR: invalid configuration '%s' data processing driver requested %i but digitizer only supports %i\n", msg, *requested, *proposed);

	return -1;
}


static int drvPadUdpCommStop(void *arg)
{
    workLoop = 0;
    epicsEventWait(shutdownEvent);
    if(drvPadUdpCommDebug) epicsPrintf("drvPadUdpComm: Stop Listener Thread\n");

    return 0;
}

void
drvPadUdpCommPrefsInit(DrvPadUdpCommPrefs p)
{
	p->nsamples          = drvPadUdpCommGetNsamplesDefault();
	p->d32               = -1;
	p->nsamples_dynamic  = -1;
	p->nchannels_dynamic = -1;
	p->col_major         = -1;
}

static long
padUdpCommInit(void)
{
int            err,i,retry;
epicsTimeStamp ts;
UdpCommPkt     rpkt;
PadReply       rply;
uint8_t        flags_sup_on, flags_sup_off;
DrvPadUdpCommPrefsRec prefs;

	if ( !drvPadUdpCommHasSetup ) {
		epicsPrintf("drvPadUdpComm -- peer not set; call drvPadUdpCommSetup(peer) first\n");
		goto bail;
	}

	/* Check if epicsTimeGetEvent() works; on post 3.14.9 (bundled generalTime) we must
	 * use event (-1) (epicsTimeEventBestTime) but this requires evrTimeGet to be modified.
	 * The following call will fail on post-3.14.9 with an unmodified evrTimeGet().
	 */

#ifdef ENSURE_EVR_INITED
	/* make sure EVR is initialized by explicitly calling init routine here */
	evrInitialize();
	/* wait for some time to make sure first events have been received      */
	epicsThreadSleep(1.0);
#endif

	if ( epicsTimeGetEvent(&ts, TS1_OR_4_TIME) ) {
		epicsPrintf("drvPadUdpComm -- WARNING: epicsTimeGetEvent() failed; this is probably\n"
		            "                 because evrTime.c has not been patched for bundled generalTime\n"
		            "                 contact Steph Allison, Kukhee Kim or Till Straumann\n");
	}

	wavBufInit();

	if ( (drvPadUdpCommSd = io.open(drvPadUdpCommPort-1)) < 0 ) {
		epicsPrintf("drvPadUdpComm -- unable to create socket: %s\n",
			strerror(-drvPadUdpCommSd));
		goto bail;
	}

	if ( (err=io.connect(drvPadUdpCommSd, drvPadUdpCommPeerAddr, drvPadUdpCommPort)) ) {
		epicsPrintf("drvPadUdpComm -- unable to connect socket: %s\n",
			strerror(-err));
		if ( -ENOTCONN == err )
			epicsPrintf("probably a failed ARP lookup!\n");
		io.close(drvPadUdpCommSd);
		drvPadUdpCommSd = -1;
		goto bail;
	}

	for ( retry=0; retry <= drvPadUdpCommRetryQuery; retry++ ) {
		if ( retry ) {
			fprintf(stderr,"No digitizer replied to feature query;\n");
			fprintf(stderr,"Waiting 30s (%i of %i) for any digitizer to come on-line...\n", retry, drvPadUdpCommRetryQuery);
			sleep( 30 );
		}

		/* Query digitizer driver for supported data formats -- don't care about timestamps */
		for ( i = 0; i<MAX_BPM; i++ ) {
			rpkt = 0;
			err = io.padIoReq(drvPadUdpCommSd, i, PADCMD_SQRY, drvPadUdpCommGetXid(), 0, 0, 0, &rpkt, 200 /*ms*/);
			if ( 0 == err && rpkt )
				goto got_reply;
			io.free(rpkt);
		}

	}

got_reply:
	if ( rpkt ) {
		rply = io.bufptr(rpkt);
		flags_sup_on  = rply->strm_sqry_sup_on;
		flags_sup_off = rply->strm_sqry_sup_off;
		io.free(rpkt);
	} else {
		epicsPrintf("drvPadUdpComm -- query of digitizer features failed: %s\n", strerror(-err));
		flags_sup_on = flags_sup_off = 0;
	}

	drvPadUdpCommPrefsInit( &prefs );

	if ( flags_sup_on == 0 && flags_sup_off == 0 ) {
		/* likely the digitizer does not (yet) implement the new protocol
		 * features; take wild guesses...
		 */
	} else {
		/* construct proposed settings based on what the digitizer supports */
		int le_sup = isbe(); /* init so the test below fails */
		set_pref(PADCMD_STRM_FLAG_32, flags_sup_on, flags_sup_off, &prefs.d32, "D32");
		set_pref(PADCMD_STRM_FLAG_C1, flags_sup_on, flags_sup_off, &prefs.nchannels_dynamic, "C1");
		set_pref(PADCMD_STRM_FLAG_CM, flags_sup_on, flags_sup_off, &prefs.col_major, "CM");
		set_pref(PADCMD_STRM_FLAG_LE, flags_sup_on, flags_sup_off, &le_sup, "LE");

		/* Verify that the digitizer can handle our endianness */
		if ( le_sup >= 0 ) {
			/* can't handle both */
			if ( !! isbe() != ! le_sup ) {
				epicsPrintf("drvPadUdpComm - FATAL ERROR: digitizer cannot supply samples with our endianness!\n");
				goto bail;
			}
		}
	}

	drvPadUdpCommPrefs = prefs;

	/* Ask the callbacks for preferences based on our proposal */
	if ( drvPadUdpCommCallbacks.get_prefs )
		drvPadUdpCommCallbacks.get_prefs( &drvPadUdpCommPrefs );

	if ( check_pref( &prefs.d32, &drvPadUdpCommPrefs.d32, 0, "D32" ) < 0 ) {
		goto bail;
	}
	if ( check_pref( &prefs.nchannels_dynamic,
	                 &drvPadUdpCommPrefs.nchannels_dynamic,
	                 0,
		             "SINGLE CHANNEL MODE" ) < 0 ) {
		goto bail;
	}
	if ( check_pref( &prefs.col_major,
	                 &drvPadUdpCommPrefs.col_major,
	                 1,
		             "COLUMN-MAJOR LAYOUT" ) < 0 ) {
		goto bail;
	}

	strmCmdInit(drvPadUdpCommPrefs.d32, drvPadUdpCommPrefs.col_major);

	simInit();

	for ( i=0; i<MAX_BPM; i++ ) {
		scanIoInit(&scanlist[i]);
		scanIoInit(&dScanlist[i]);
	}

        shutdownEvent = epicsEventMustCreate(epicsEventEmpty);
        workLoop      = 1;
	epicsThreadCreate(
			"drvPadUdpCommListener",
			epicsThreadPriorityMax,
			4*epicsThreadGetStackSize(epicsThreadStackBig),
			drvPadUdpCommListener,
			0);
			
	epicsThreadCreate(
			"drvPadUdpCommWatchdog",
			epicsThreadPriorityHigh,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			drvPadUdpCommWatchdog,
			0);

       epicsAtExit3((epicsExitFunc) drvPadUdpCommStop, (void*) NULL, "drvPadUdpCommStop");      

	if ( drvPadUdpCommCallbacks.fid_process_install )
		drvPadUdpCommCallbacks.fid_process_install();
	else
		fidTimeInstall_generic();

	return 0;

bail:
	fflush(stdout);
	fflush(stderr);
	abort();
	return -1;
}

long
drvPadUdpCommGetIointInfo(int cmd, struct dbCommon *precord, IOSCANPVT *ppvt)
{
DrvPadUdpCommDpvt dpvt = (DrvPadUdpCommDpvt)precord->dpvt;
	if ( dpvt->channel >= MAX_BPM )
		return -1;
	if ( menuPriorityHIGH == precord->prio ) {
		*ppvt = scanlist[dpvt->channel];
	} else {
		*ppvt = dScanlist[dpvt->channel];
	}
	return 0;
}

int
drvPadUdpCommDpvtInit(DrvPadUdpCommDpvt padUdpCommPart, unsigned channel)
{
	padUdpCommPart->channel = channel;
	return 0;
}

static struct {
	long		number;
	DRVSUPFUN	report;
	DRVSUPFUN	init;
} drvPadUdpComm = {
	2,
	padUdpCommReport,
	padUdpCommInit
};
epicsExportAddress(drvet,drvPadUdpComm);

static const struct iocshArg setup_args[] = {
	{
	"peer <ip:port> | VME",
	/* iocshArgPersistentString is broken (3.14.12) -
	 * it doesn't check for an empty string before
	 * duplicating :-(
	 */
	iocshArgString
	},
	{
	"callbacks",
	iocshArgString
	},
	{
	"io_ops",
	iocshArgString
	}
};

static const struct iocshArg *setup_bloatp[] = {
	&setup_args[0],
	&setup_args[1],	
	&setup_args[2],	
};

static const struct iocshFuncDef drvPadUdpCommSetupDesc = {
	"drvPadUdpCommSetup",
	sizeof(setup_bloatp)/sizeof(setup_bloatp[0]),
	setup_bloatp
};

static int
udpCommSetup(const char *arg, uint32_t *peer_ip_p, int *peer_port_p)
{
char *col, *str;
struct in_addr ina;

	if ( !(str=epicsStrDup(arg)) ) {
		epicsPrintf("udpCommSetup: not enough memory for epicsStrDup\n");
		return -1;
	}

	if ( (col = strchr(str, ':')) )
		*col++=0;

	ina.s_addr = *peer_ip_p = INADDR_NONE;

	if ( hostToIPAddr(str, &ina) ) {
		epicsPrintf("udpCommSetup: invalid IP address '%s'\n", str);
	} else {
		*peer_ip_p = ina.s_addr;
	}
	if ( col && 1 != sscanf(col, "%i", peer_port_p) ) {
		epicsPrintf("drvPadUdpCommSetup: invalid port number:  '%s'\n", col);
		*peer_port_p = PADPROTO_PORT;
	}

	if ( 0 == *peer_port_p )
		*peer_port_p = PADPROTO_PORT;

	free(str);

	return 0;
}

void
drvPadUdpCommSetup(const char *arg, DrvPadUdpCommCallbacks callbacks, DrvPadUdpCommIO io_ops)
{
	if ( !callbacks ) {
		epicsPrintf("drvPadUdpCommSetup: need pointer to callback info!\n");
		return;
	}

	/*
	 * cache callback info locally -- in case their struct lives
	 * on the stack...
	 */
	drvPadUdpCommCallbacks = *callbacks;

	if ( io_ops )
		io = *io_ops;

	drvPadUdpCommHasSetup = io.setup ?  ! io.setup(arg, &drvPadUdpCommPeerAddr, &drvPadUdpCommPort) : 1;
}

const void *drvPadUdpCommRegistryId = (void*)&drvPadUdpCommRegistryId;

void
drvPadUdpCommSetupFunc(const iocshArgBuf *args)
{
void *cbs;
void *io_ops = 0;

	if ( !args[0].sval ) {
		epicsPrintf("usage: drvPadUdpCommSetup(\"peer_ip:port | VME\", callbacks)\n");
		return;
	}
	if  ( ! (cbs = registryFind((void*)drvPadUdpCommRegistryId, args[1].sval)) ) {
		epicsPrintf("callbacks: '%s' not found; unable to setup\n",args[1].sval);
		return;
	}
	if ( args[2].sval && *args[2].sval ) {
		if ( ! (io_ops = registryFind((void*)drvPadUdpCommRegistryId, args[2].sval)) ) {
			epicsPrintf("io_ops: '%s' not found; unable to setup\n",args[2].sval);
			return;
		}
	}

	drvPadUdpCommSetup(args[0].sval, cbs, io_ops);
}


DrvPadUdpCommHWTime
drvPadUdpCommMaxCookReset(int doit)
{
DrvPadUdpCommHWTime old = drvPadUdpCommMaxCook;
	if ( doit )
		drvPadUdpCommMaxCook = 0;
	return old;
}

static const struct iocshArg mctr_args[] = {
	{
	"do_reset_if_nonzero",
	iocshArgInt
	},
};

static const struct iocshArg *mctr_bloatp[] = {
	&mctr_args[0],
};

static const iocshFuncDef mctrDesc = {
	"drvPadUdpCommMaxCookReset",
	sizeof(mctr_bloatp)/sizeof(mctr_bloatp[0]),
	mctr_bloatp
};

static void mctrFunc(const iocshArgBuf *args)
{
	epicsPrintf("%d\n", (unsigned)drvPadUdpCommMaxCookReset(args[0].ival));
}

static void
drvPadUdpCommRegistrar(void)
{
	iocshRegister(&drvPadUdpCommSetupDesc, drvPadUdpCommSetupFunc);
	iocshRegister(&mctrDesc, mctrFunc);
}
epicsExportRegistrar(drvPadUdpCommRegistrar);
epicsExportAddress(int, drvPadUdpCommDebug);
