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
#error "EPICS > 3.14.9 -- There must have something changed in the EVR code -- I don't know the appropriate event code"
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

#define RAD ((PI)/180.0)

#ifdef __PPC__
uint32_t	roundtripMax = 0;
uint32_t	roundtripAvg = 0;

#endif

void
padUdpCommPostRaw(UdpCommPkt pkt, PadDataKind kind);

uint32_t	drvPadUdpCommDebug = 0;

volatile DrvPadUdpCommHWTime drvPadUdpCommMaxCook = 0;

#if MAX_BPM > 31
#error "too many channels -- don't fit in 32-bit mask"
#endif
volatile uint32_t bpmUpToDate[NUM_DATA_KINDS] = { 0, 0, 0};
volatile uint32_t drvPadUdpCommBpmInUse    = 0;

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

	padStrmCmd.req.version = PADPROTO_VERSION2;
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
	}
	padStrmCmdLock = epicsMutexMustCreate();
}

int
drvPadUdpCommStrmStopReq(int channel)
{
int key, rval;
epicsTimeStamp ts;

	if ( channel < 0 || channel >= MAX_BPM )
		return -1;

	key = epicsInterruptLock();
		drvPadUdpCommBpmInUse &= ~(1<<channel);
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
				htonl(ts.secPastEpoch), htonl(ts.nsec),
				0,
				0,
				drvPadUdpCommTimeout);

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

	key = epicsInterruptLock();
		drvPadUdpCommBpmInUse |= (1<<channel);
	epicsInterruptUnlock(key);

	return rval;
}

int
drvPadUdpCommStrmSetNSamples(int channel, int nsamples)
{
int rval;

	if ( ! drvPadUdpCommPrefs.nsamples_dynamic ) {
		epicsPrintf("drvPadUdpCommStrmSetNSamples: client-driver forbids changing sample number\n");
		return -1;
	}

	if ( nsamples > 170 ) {
		/* wouldn't fit into std. ethernet packet */
		return -1;
	}

	if ( channel < 0 || channel >= MAX_BPM )
		return -1;

	if ( channel < 0 && nsamples <= 0 )
		return -1;

	if ( nsamples <= 0 ) {
		epicsMutexLock(padStrmCmdLock);
		rval = ntohl(padStrmCmd.cmd[channel].nsamples);
		epicsMutexUnlock(padStrmCmdLock);
		return rval;
	}

	nsamples = htonl(nsamples);

	epicsMutexLock(padStrmCmdLock);
	if ( channel < 0 ) {
		for ( channel = 0; channel < MAX_BPM; channel++ )
			padStrmCmd.cmd[channel].nsamples = nsamples;
		rval = 0;
	} else {
		rval = ntohl(padStrmCmd.cmd[channel].nsamples);
		padStrmCmd.cmd[channel].nsamples = nsamples;
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

PadReply
padReplyFind(void *data_p)
{
	return data_p ? (PadReply)io.bufptr(data_p) : 0;
}

int
padDataFree(WavBuf wb)
{
	io.free(wb->usrData);
	return 0;
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

static void
drvPadUdpCommListener(void *arg)
{
int         sd;
UdpCommPkt  pkt = 0;
WavBuf      pb  = 0;
PadReply    rply;
PadDataKind kind;
unsigned    chan;
unsigned    nsamples;
int         layout_cm;
int         cook_stat;
int         key;
epicsTimeStamp         now;
DrvPadUdpCommCallbacks cb = &drvPadUdpCommCallbacks;
DrvPadUdpCommHWTime    tDiff;

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
		nsamples = PADRPLY_STRM_NSAMPLES(rply);

		if ( chan >= MAX_BPM ) {
			/* drop invalid channels */
			errlogPrintf("drvPadUdpCommListener: invalid BPM channel #%i\n", chan);
	continue;
		}

		drvPadUdpCommHWTimeBaseline[chan]  = tDiff;
		drvPadUdpCommPktTimeBaseline[chan] = ntohl(rply->xid);

		/* Layout (column-major or row-major) */
		layout_cm = (rply->strm_cmd_flags & PADCMD_STRM_FLAG_CM );

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
			errlogPrintf("STRM: C %2u N %3u K %1u TS: 0x%08"PRIx32" XID: 0x%08"PRIx32"\n",
				chan, nsamples, kind, ntohl(rply->timestampLo), ntohl(rply->xid));
		}

		cook_stat = 1;

		/* TODO: find out what kind of cycle this is */
		switch ( kind ) {

				case PadDataBpm:
				case PadDataCalRed:
				case PadDataCalGrn:
				{
					if ( cb->cook ) {
						cook_stat = cb->cook(rply, nsamples, layout_cm, kind, cb->cookClosure);
					} else {
						cook_stat = -1;
					}
				}
				break;

				default:
					errlogPrintf("drvPadUdpCommListener: bad data kind (0x%x) from channel %i\n", kind, chan);
					cook_stat = -1;
				break;
		}

		if ( cook_stat >= 0 ) {
			/* Pet up-to-date status */
			key = epicsInterruptLock();
				bpmUpToDate[kind] |= (1<<chan);
			epicsInterruptUnlock(key);
		}

		if ( cook_stat )
			continue;

		if ( ! pb ) {
			if ( !(pb = wavBufAlloc()) ) {
				/* Just drop */
				epicsPrintf("drvPadUdpCommListener: unable to get waveform buf header -- configure more!!\n");
				continue;
			}
			pb->free = padDataFree; 
		}
		if ( rply->strm_cmd_flags & PADCMD_STRM_FLAG_CM ) {
			pb->flags = WavBufFlagCM;
		} else {
			pb->flags = 0;
		}
		pb->type    = (rply->strm_cmd_flags & PADCMD_STRM_FLAG_32) ? WavBufInt32 : WavBufInt16;
		pb->m       = 4;
		pb->n       = nsamples;
		pb->usrData = pkt;
		pb->data    = rply->data;
		pb->ts.secPastEpoch = ntohl(rply->timestampHi);
		pb->ts.nsec         = ntohl(rply->timestampLo);

		/* send buffer for waveforms to pick up */
		if ( 0 == wavBufPost(chan, kind, pb) ) {
			pkt = 0;
			pb  = 0;
		}

		/* Scan lists only on the beam pulse. Waveforms
		 * listening to the cal pulses will also be on
		 * the same scanlist.
		 */
		if ( PadDataBpm == kind ) {
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
		uint32_t    timedout[NUM_DATA_KINDS], t, s, sa;

		/*  s: bitfield of all channels that are in use and are timed out
         *     for at least one kind of data.
		 * sa: bitfield of all channels that are in use and are timed out
         *     for all kinds of data.
         */
		s   = 0;
		sa  = 0xffffffff;
		key = epicsInterruptLock();
			for ( j=0; j<NUM_DATA_KINDS; j++ ) {
				if ( (cb->watchKinds & (1<<j)) ) {
					timedout[j] = drvPadUdpCommBpmInUse & ~bpmUpToDate[j];
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
					for ( j=0; j<NUM_DATA_KINDS; j++ )
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
	padSimulateCmd.req.version = PADPROTO_VERSION2;
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
int err,i;
int isVme;

	if ( !drvPadUdpCommPeer ) {
		epicsPrintf("drvPadUdpComm -- peer not set; call drvPadUdpCommSetup(peer) first\n");
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

	if ( (drvPadUdpCommSd = io.open(drvPadUdpCommPort-1)) < 0 ) {
		epicsPrintf("drvPadUdpComm -- unable to create socket: %s\n",
			strerror(-drvPadUdpCommSd));
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

void
padUdpCommPostRaw(UdpCommPkt pkt, PadDataKind kind)
{
WavBuf   pb;
PadReply rply;
unsigned chan;
unsigned nsamples;

	if ( !pkt )
		return;

	rply = padReplyFind(pkt);

	chan     = rply->chnl;
	nsamples = PADRPLY_STRM_NSAMPLES(rply);

	if ( !(pb = wavBufAlloc()) ) {
		/* Just drop */
		epicsPrintf("drvPadUdpCommListener: unable to get waveform buf header -- configure more!!\n");
		io.free(pkt);
		return;
	}
	pb->type = (rply->strm_cmd_flags & PADCMD_STRM_FLAG_32) ? WavBufInt32 : WavBufInt16;
	pb->free = padDataFree; 
	if ( rply->strm_cmd_flags & PADCMD_STRM_FLAG_CM ) {
		pb->flags = WavBufFlagCM;
	} else {
		pb->flags = 0;
	}
	pb->m = 4;
	pb->n = nsamples;
	pb->usrData = pkt;
	pb->data    = rply->data;
	pb->ts.secPastEpoch = ntohl(rply->timestampHi);
	pb->ts.nsec         = ntohl(rply->timestampLo);

	/* send buffer for waveforms to pick up */
	if ( 0 == wavBufPost(chan, kind, pb) ) {
			pkt = 0;
			pb  = 0;
	} else {
		/* This releases the packet, too */
		wavBufFree(pb);
	}
}
