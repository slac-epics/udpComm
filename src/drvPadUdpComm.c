#include <udpComm.h>
#include "padProto.h"
#include <padStream.h>

#include <errno.h>

#include <epicsInterrupt.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <epicsVersion.h>
#include <registryFunction.h>
#include <errlog.h>
#include <drvSup.h>
#include <iocsh.h>
#include <registry.h>

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


uint32_t	drvPadUdpCommDebug = 0;

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
volatile DrvPadUdpCommHWTime drvPadUdpCommPktTimeBaseline[MAX_BPM] = { 0 };

static DrvPadUdpCommIORec io = {
	open:     udpCommSocket,
	close:    udpCommClose,
	connect:  udpCommConnect,
	send:     udpCommSend,
	recv:     udpCommRecv,
	bufptr:   udpCommBufPtr,
	alloc:    udpCommAllocPacket,
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

	padStrmCmd.req.version = PADPROTO_VERSION3;
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
uint32_t       old_val;

	if ( channel < 0 || channel >= MAX_BPM )
		return -1;

	key = epicsInterruptLock();
		old_val = (drvPadUdpCommChannelsInUseMask & (1<<channel));
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
				PADCMD_STOP,
				drvPadUdpCommGetXid(),
				htonl(ts.secPastEpoch), htonl(ts.nsec),
				0,
				0,
				drvPadUdpCommTimeout);

	if ( rval ) {
		key = epicsInterruptLock();
			drvPadUdpCommChannelsInUseMask |= old_val;
		epicsInterruptUnlock(key);
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
					PADCMD_STRM,
					drvPadUdpCommGetXid(),
					htonl(ts.secPastEpoch), htonl(ts.nsec),
					&scmd,
					0,
					drvPadUdpCommTimeout);

	if ( rval ) {
		epicsPrintf("drvPadUdpCommStrmStartReq: Unable to start stream: %s\n", strerror(-rval));
		return rval;
	}

	key = epicsInterruptLock();
		drvPadUdpCommChannelsInUseMask |= (1<<channel);
	epicsInterruptUnlock(key);

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
char *drvPadUdpCommPeer   = 0;
uint32_t drvPadUdpCommPeerAddr = INADDR_NONE;

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
	return 0;
}

uint32_t
drvPadUdpCommGetXid()
{
#if defined(__PPC__) || defined(__i386__)
	/* using timebase can be useful as a baseline for timing purposes */
	return drvPadUdpCommHWTime();
#else

static volatile uint32_t xid = 0;
uint32_t rval;
int      key;

	key = epicsInterruptLock();
	rval = xid++;
	epicsInterruptUnlock(key);
	return rval;
#endif
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
	
	return io.padIoReq(
				drvPadUdpCommSd,
				channel,
				scmd.type,
				xid,
				htonl(ts.secPastEpoch), htonl(ts.nsec),
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

	if ( !pkt )
		return rval;

	rply     = padReplyFind(pkt);

	chan     = rply->chnl;

	if ( chan >= WAV_BUF_NUM_SLOTS || (unsigned)kind >= WAV_BUF_NUM_KINDS ) {
		epicsPrintf("drvPadUdpCommPostRaw: invalid chan/kind (%u/%u), dropping...\n", chan, kind);
		goto bail;
	}

	if ( (pb = wbStaged[chan][kind]) ) {
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
		if ( rply->xid != ((PadReply)((uintptr_t)pb->segs.data[pb->segs.nsegs-1] - (uintptr_t)((PadReply)0)->data))->xid ) {
			/* fragment already part of a new stream packet */
			wavBufFree( pb );
			pb = wbStaged[chan][kind] = 0;
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
	}

	idx = PADRPLY_STRM_CMD_IDX_GET( rply->strm_cmd_idx );

	if ( idx >= WAV_BUF_NUM_SEGS ) {
		epicsPrintf("drvPadUdpCommPostRaw: number of segments too big; dropping\n");
		goto bail;
	}

	if ( idx + 1 > pb->segs.nsegs )
		pb->segs.nsegs = idx + 1;
	pb->segs.data[idx] = rply->data;
	pkt                = 0;

	if ( ! (PADRPLY_STRM_CMD_IDX_MF & rply->strm_cmd_idx) ) {
		/* found the last fragment */
		pb->usrData = (void*)( (uintptr_t)pb->usrData - FRAG_MAX + idx + 1 - 1 );
		pb->n       = (idx + 1) * PADRPLY_STRM_NSAMPLES(rply) / pb->m;
	} else {
		pb->usrData = (void*)( (uintptr_t)pb->usrData - 1 );
	}

	if ( 0 == (uintptr_t)pb->usrData ) {
		/* All fragments assembled */

		/* send buffer for waveforms to pick up */
		if ( 0 == wavBufPost(chan, kind, pb) ) {
			rval = 0;
		} else {
			/* This releases the packet, too */
			wavBufFree(pb);
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
DrvPadUdpCommCallbacks cb = &drvPadUdpCommCallbacks;
DrvPadUdpCommHWTime    tDiff;
int         bad_version_count = 0;

	if ( cb->init )
		cb->init(cb);

	/* Open a socket for incoming streamed data */

	sd = io.open(drvPadUdpCommPort + 1);

	if ( sd < 0 ) {
		epicsPrintf("drvPadUdpCommListener -- unable to create socket: %s\n",
			strerror(-sd));
		return;
	}

	while ( 1 ) {
		/* release old buffer */
		io.free(pkt);

		/* keep sending start command until we receive something */
		while ( !( pkt = io.recv(sd, drvPadUdpCommTimeout)) )
			drvPadUdpCommSendStartAll(0);

		tDiff    = drvPadUdpCommHWTime();

		rply     = padReplyFind(pkt);

		if ( PADPROTO_VERSION3 != rply->version ) {
			bad_version_count++;
			if ( (bad_version_count & 0xff) == 1 ) {
				errlogPrintf("drvPadUdpCommListener: dropped %u unsupported version %u packets (need %u)\n", bad_version_count, rply->version, PADPROTO_VERSION3);
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
		drvPadUdpCommPktTimeBaseline[chan] = ntohl(rply->xid);

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

		if ( cb->cook ) {
			cook_stat = cb->cook(rply, nsamples, layout_cm, kind, cb->cookClosure);
		} else {
			cook_stat = -1;
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
					if (0 == drvPadUdpCommPostRaw( pkt, new_kind ))
						posted++;
					pkt = 0;
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
		tDiff   = drvPadUdpCommHWTime() - tDiff;

		if ( tDiff > drvPadUdpCommMaxCook )
			drvPadUdpCommMaxCook = tDiff;
	}
}

static void
drvPadUdpCommWatchdog(void *arg)
{
int i, err = 0;

DrvPadUdpCommCallbacks cb = &drvPadUdpCommCallbacks;

	while ( 1 ) {
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
	padSimulateCmd.req.version = PADPROTO_VERSION3;
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

static long
padUdpCommInit(void)
{
int            err,i;
int            isVme;
epicsTimeStamp ts;

	if ( !drvPadUdpCommPeer ) {
		epicsPrintf("drvPadUdpComm -- peer not set; call drvPadUdpCommSetup(peer) first\n");
		return -1;
	}

	/* Check if epicsTimeGetEvent() works; on post 3.14.9 (bundled generalTime) we must
	 * use event (-1) (epicsTimeEventBestTime) but this requires evrTimeGet to be modified.
	 * The following call will fail on post-3.14.9 with an unmodified evrTimeGet().
	 */

	if ( epicsTimeGetEvent(&ts, TS1_OR_4_TIME) ) {
		epicsPrintf("drvPadUdpComm -- epicsTimeEventGetEvent() failed; this is probably\n"
		            "                 because evrTime.c has not been patched for bundled generalTime\n"
		            "                 contact Steph Allison, Kukhee Kim or Till Straumann\n");
		return -1;
	}

	wavBufInit();

	isVme = ( 0 == strcmp(drvPadUdpCommPeer, "VME") );

	drvPadUdpCommPrefs.nsamples         = drvPadUdpCommGetNsamplesDefault();
	drvPadUdpCommPrefs.nsamples_dynamic = 1;
	drvPadUdpCommPrefs.col_major        = isVme;

	if ( drvPadUdpCommCallbacks.get_prefs )
		drvPadUdpCommCallbacks.get_prefs( &drvPadUdpCommPrefs );

	if ( !drvPadUdpCommPrefs.col_major && isVme ) {
		epicsPrintf("drvPadUdpComm -- invalid configuration; client driver asks for row-major layout but VME digitizer only supports col-major\n");
		return -1;
	}

	if ( ! isVme && INADDR_NONE == drvPadUdpCommPeerAddr ) {
		epicsPrintf("drvPadUdpComm -- invalid or no peer ID; call drvPadUdpCommSetup(peer) with 'peer' a string in 'dot' notation (or \"VME\")...\n");
		return -1;
	}

	if ( isVme ) {
		if ( ! &drvPadVmeCommIO || !drvPadVmeCommIO ) {
			epicsPrintf("drvPadUdpComm -- no VME IO methods; is the vmeDigiComm driver linked to your application?\n");
			return -1;
		} else {
			io = *drvPadVmeCommIO;
		}
	}

	if ( (drvPadUdpCommSd = io.open(drvPadUdpCommPort-1)) < 0 ) {
		epicsPrintf("drvPadUdpComm -- unable to create socket: %s\n",
			strerror(-drvPadUdpCommSd));
		return -1;
	}

	strmCmdInit(drvPadUdpCommPrefs.d32, drvPadUdpCommPrefs.col_major);

	simInit();

	for ( i=0; i<MAX_BPM; i++ ) {
		scanIoInit(&scanlist[i]);
		scanIoInit(&dScanlist[i]);
	}

	if ( (err=io.connect(drvPadUdpCommSd, drvPadUdpCommPeerAddr, drvPadUdpCommPort)) ) {
		epicsPrintf("drvPadUdpComm -- unable to connect socket: %s\n",
			strerror(-err));
		if ( -ENOTCONN == err )
			epicsPrintf("probably a failed ARP lookup!\n");
		io.close(drvPadUdpCommSd);
		drvPadUdpCommSd = -1;
		return -1;
	}

	epicsThreadCreate(
			"drvPadUdpCommListener",
			epicsThreadPriorityHigh,
			4*epicsThreadGetStackSize(epicsThreadStackBig),
			drvPadUdpCommListener,
			0);
			
	epicsThreadCreate(
			"drvPadUdpCommWatchdog",
			epicsThreadPriorityHigh,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			drvPadUdpCommWatchdog,
			0);

	if ( drvPadUdpCommCallbacks.fid_process_install )
		drvPadUdpCommCallbacks.fid_process_install();
	else
		fidTimeInstall_generic();

	return 0;
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

static const struct iocshArg args[] = {
	{
	"peer <ip:port> | VME",
	/* iocshArgPersistentString is broken (3.14.8) -
	 * it doesn't check for an empty string before
	 * duplicating :-(
	 */
	iocshArgString
	},
	{
	"callbacks",
	iocshArgString
	}
};

static const struct iocshArg *bloatp[] = {
	&args[0],
	&args[1],	
	0
};

struct iocshFuncDef drvPadUdpCommSetupDesc = {
	"drvPadUdpCommSetup",
	2,
	bloatp
};

void
drvPadUdpCommSetup(const char *arg, DrvPadUdpCommCallbacks callbacks)
{
char *col, *str;
struct in_addr ina;

	if ( !callbacks ) {
		epicsPrintf("drvPadUdpCommSetup: need pointer to callback info!\n");
		return;
	}

	/*
	 * cache callback info locally -- in case their struct lives
	 * on the stack...
	 */
	drvPadUdpCommCallbacks = *callbacks;

	if ( !(str=epicsStrDup(arg)) ) {
		epicsPrintf("drvPadUdpCommSetup: not enough memory for epicsStrDup\n");
		return;
	}

	if ( (col = strchr(str, ':')) )
		*col++=0;

	ina.s_addr = drvPadUdpCommPeerAddr = INADDR_NONE;

	if ( strcmp(str,"VME") && hostToIPAddr(str, &ina) ) {
		epicsPrintf("drvPadUdpCommSetup: invalid IP address '%s'\n",
			str);
		free(str);
	} else {
		drvPadUdpCommPeerAddr = ina.s_addr;
		drvPadUdpCommPeer     = str;
	}
	if ( col && 1 != sscanf(col, "%i", &drvPadUdpCommPort) ) {
		epicsPrintf("drvPadUdpCommSetup: invalid port number:  '%s'\n", col);
		drvPadUdpCommPort = PADPROTO_PORT;
	}

	if ( 0 == drvPadUdpCommPort )
		drvPadUdpCommPort = PADPROTO_PORT;

}

const void *drvPadUdpCommRegistryId = (void*)&drvPadUdpCommRegistryId;

void
drvPadUdpCommSetupFunc(const iocshArgBuf *args)
{
void *cbs;
	if ( !args[0].sval ) {
		epicsPrintf("usage: drvPadUdpCommSetup(\"peer_ip:port | VME\", callbacks)\n");
		return;
	}
	if  ( ! (cbs = registryFind((void*)drvPadUdpCommRegistryId, args[1].sval)) ) {
		epicsPrintf("callbacks: '%s' not found; unable to setup\n",args[1].sval);
		return;
	}

	drvPadUdpCommSetup(args[0].sval, cbs);
}

static void
drvPadUdpCommRegistrar(void)
{
	iocshRegister(&drvPadUdpCommSetupDesc, drvPadUdpCommSetupFunc);
}
epicsExportRegistrar(drvPadUdpCommRegistrar);
