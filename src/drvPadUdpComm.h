#ifndef DRV_PAD_UDP_COMM_H
#define DRV_PAD_UDP_COMM_H

#include <stdint.h>
#include <padProto.h>
#include <padStream.h>
#include <dbCommon.h>
#include <dbScan.h>
#include <aiRecord.h>
#include <epicsTime.h>

/* Max # BPMs supported per IOC */
#define MAX_BPM 25

#if 0
#define PAD_OVERRANGE(x) abs((x)) > CLIP_THRESHOLD
#define CLIP_THRESHOLD	32000
#else
/* PAD has ADC overrange flag encoded in data LSB */
#define PAD_OVERRANGE(x)	((x)&1)
#endif

#ifndef NumberOf
#define NumberOf(arr) (sizeof((arr))/sizeof((arr)[0]))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DrvPadUdpCommCallbacksRec_ *DrvPadUdpCommCallbacks;

/*
 * Obtain stream settings;
 * the callback is allowed to only partly modify
 * the struct, i.e., drvPadUdpComm initializes it
 * to proposed values which may be modified by
 * the client driver.
 */
typedef struct DrvPadUdpCommPrefsRec_ {
	int nsamples;
	int d32;
	int nsamples_dynamic;
	int nchannels_dynamic;      /* whether to allow users to switch to single- (ADC) channel mode */
	int col_major;
} DrvPadUdpCommPrefsRec, *DrvPadUdpCommPrefs;

/* Initialize a Prefs Rec to default values (everything OK) */
void
drvPadUdpCommPrefsInit(DrvPadUdpCommPrefs p);

/* 'cook' callback may return either of these
 * in order to have the raw data posted to 'wavBuf'
 * of 'kind'.
 */
/* Post raw data but skip scanning the records
 * which pick up the cooked data.
 */
/* These three macros are for INTERNAL use only */
#define PAD_UDPCOMM_COOK_STAT_DEBUG_MASK (~(0xff))
#define PAD_UDPCOMM_COOK_STAT_DEBUG_FLG_NOSCAN 0x100
#define PAD_UDPCOMM_COOK_STAT_DEBUG_FLG_DOSCAN 0x200
/* Three macros above are for INTERNAL use only */

#define PAD_UDPCOMM_COOK_STAT_DEBUG_NOSCAN(kind)\
	( PAD_UDPCOMM_COOK_STAT_DEBUG_FLG_NOSCAN | ((kind)&0xff))
/* Post raw data but have the records scanned
 * to pick up the cooked data.
 */
#define PAD_UDPCOMM_COOK_STAT_DEBUG_DOSCAN(kind)\
	( PAD_UDPCOMM_COOK_STAT_DEBUG_FLG_DOSCAN | ((kind)&0xff))

/* Set 'data received' flag and do request scanlist to be processed    */
#define PAD_UDPCOMM_COOK_STAT_OK             0
/* Set 'data received' flag but don't request scanlist to be processed */
#define PAD_UDPCOMM_COOK_STAT_NOSCAN         1
/* Do not set 'data received' flag, do not request scanlist to be
 * processed and do not post raw data
 */
#define PAD_UDPCOMM_COOK_STAT_NOSCAN_NOPOST (-1)

typedef struct DrvPadUdpCommCallbacksRec_ {
	/* 'init' is executed from the UDP comm listener task which
	 * also executes the 'cook' callback.
	 * The 'init' procedure may modify the 'cookClosure'
	 * argument.
	 */
	int  (*init)(DrvPadUdpCommCallbacks);
	void *cookClosure;
	int (*cook)	(PadReply rply, int nsamples, int layout_cm, int kind, void *closure);
	void *watchdogClosure;
	void (*watchdog)(int channel, int kind, void *closure);       
	unsigned    watchKinds;
	/* check and override preferences; non-zero return values
	 * reserved for future extension;
	 */
	int (*get_prefs)(DrvPadUdpCommPrefs);
	/* generate simulated position */
	int (*sim_pos)(int32_t *vals_p, float x, float y, float q, float phi, float psi, void *parms);
	void (*fid_process_install)(void);
} DrvPadUdpCommCallbacksRec;

/* socket descriptor for sending requests to PAD */
extern int drvPadUdpCommSd;
/* PAD IP and port */
extern char *drvPadUdpCommPeer;
extern uint32_t  drvPadUdpCommPeerAddr;
extern int drvPadUdpCommPort;
/* timeout (in ms) when waiting for a reply or streamed data */
extern int drvPadUdpCommTimeout;
extern double drvPadUdpCommStrmTimeout;
/* bitmask of channels that are currently in-use (not 'offline') -- READ_ONLY */
extern volatile uint32_t drvPadUdpCommChannelsInUseMask;

/* wavBufGet() obtains an opaque object.
 * In order to retrieve the 'PadReply' data structure
 * you must call 'padReplyFind()'...
 */
PadReply
padReplyFind(void *data_p);

/* Create a reference to a reply packet (e.g., if the cook callback
 * has further use for it).
 * 'padReplyFree()' [below] decrements the reference count
 * and releases the reply packet buffer once the reference count
 * drops to zero.
 */
void
padReplyRef(PadReply rply);

void
padReplyFree(PadReply rply);


/* Post 'rply' to wavBuf slot 'kind'. Note that the caller
 * Must 'own' a reference to the reply packet. When this
 * is e.g., executed from a 'cook' callback then a reference
 * must explicitly created e.g., by padReplyRef( rply );
 * Ownership of the packet/reference is transferred to the
 * wavBuf code which takes care of releasing the packet
 * (even under error condition).
 */
int
drvPadUdpCommPostReply(PadReply rply, PadDataKind kind);


/* get a new 'transaction ID'; a global counter
 * is incremented atomically by this routine
 * (but you may use a different policy for the
 * transaction id that is passed to 'padRequest').
 */
uint32_t
drvPadUdpCommGetXid();

/* Send a 'start' command to one ('channel'#) or all (channel < 0)
 * PADs tagging the streamed data with transaction ID 'xid'.
 *
 * RETURNS: return code of underlying 'padRequest()' call.
 */
int
drvPadUdpCommSendStartCmd(int nsamples, uint32_t xid, int channel);

/* Convenience wrapper using drvpadUdpCommGetXid() and broadcasting
 * to all PADs using timestamp '*ts_p' (may be NULL and zero timestamps
 * are used).
 *
 * The sample number and state (on/off) can be modified via the
 * drvPadUdpCommStrmSetNSamples() routine.
 *
 * RETURNS: return code of underlying 'padRequest()' call.
 */
int
drvPadUdpCommSendStartAll(epicsTimeStamp *ts_p);

/* All device support modules using drvPadUdpCommGetIointInfo()
 * must include the struct DrvPadUdpCommDpvtRec_ at the beginning
 * of their respective DPVT structs.
 */

typedef struct DrvPadUdpCommDpvtRec_ {
	unsigned	channel;	/* BPM / channel this record represents */
} DrvPadUdpCommDpvtRec, *DrvPadUdpCommDpvt;

/* Initialize the above struct */
int
drvPadUdpCommDpvtInit(DrvPadUdpCommDpvt padUdpCommPart, unsigned channel);

/* get_ioint_info routine to put a record on the BPM scanning
 * list.
 *
 * Note that the driver maintains two scanning lists and the
 * record's PRIO field determines to which one it is added.
 *
 * The 'normal-duty' records which are scanned on every beam
 * pulse must set PRIO to HIGH.
 *
 * Records (mostly for diagnostic purposes) with PRIO = LOW
 * or MEDIUM are scanned at every beam pulse but not faster
 * than ~1Hz.
 *
 * !! DO NOT set PRIO of diagnostic waveforms to HIGH !!
 */
long
drvPadUdpCommGetIointInfo(int cmd, struct dbCommon *precord, IOSCANPVT *ppvt);

/* Request a PAD to start/stop streaming;
 * RETURNS: 0 on success; nonzero on error (invalid
 *          channel, remote request failure).
 */
int
drvPadUdpCommStrmStartReq(int channel);

int
drvPadUdpCommStrmStopReq(int channel);

/*
 * Set sample number for a particular channel (all channels if 'channel' < 0 )
 *
 * RETURNS: -1 on error, previous # of samples on success.
 *          If 'channel'<0 then zero is returned on success.
 *
 * NOTE:    routine can be used to read the current # of samples of
 *          a particular channel w/o changing it by passing nsamples=0.
 *
 *          The stream for channel (or all channels if channel=-1 is passed)
 *          MUST NOT be running while the number of samples is modified
 *          (Call fails otherwise).
 */
int
drvPadUdpCommStrmSetNSamples(int channel, int nsamples);

/*
 * Set bitmask of ADC channels to enable (PADCMD_STRM_CHANNELS_ALL
 * enables all channels). A particular channel (0..3) is enabled
 * using (1<<channel_no).
 *
 * RETURNS: -1 on error, previous channel mask on success.
 *          If 'padChannel'<0 then zero is returned on success.
 *
 * NOTE:    routine can be used to read the current ADC channel mask of
 *          a particular padChannel w/o changing it by passing adcChannels < 0.
 *
 *          The stream for padChannel (or all padChannels if padChannel=-1 is passed)
 *          MUST NOT be running while the number of ADC channels is modified
 *          (Call fails otherwise).
 */
int
drvPadUdpCommStrmSetChannels(int padChannel, int adcNChannels);

/* Normal operating mode; simulator off */
#define SIM_OFF	            0
/* Every write to the SIM PV will transmit
 * input values to the PAD simulators but
 * the PAD will not send a simulated reply back.
 * Instead, the PAD will generate a simulated
 * waveform on each HW trigger.
 */
#define SIM_HW_TRIGGERED	1

/* ('off-line' mode w/o EVR) the pad immediately
 * generates a simulated waveform as a response
 * to a simulate request.
 */
#define SIM_SW_TRIGGERED	2

int
drvPadUdpCommGetSimMode(unsigned chanl);

/* Write Simulated BPM readings to the PAD */

/* x,y, position (normalized to BPM radius)
 * q arbitrary scale. The simulator produces a
 * full scale data for (strip) values of ~200k
 *
 * -> to produce full scale at 1/2 of the BPM radius
 * 
 * RETURNS: 0 on success, nonzero on error (pad # invalid)
 */
int 
drvPadUdpCommSimulatePos(unsigned pad, int mode, float x, float y, float q, float phi, float psi, void *parms);

extern const void *drvPadUdpCommRegistryId;

typedef uint32_t DrvPadUdpCommHWTime;

DrvPadUdpCommHWTime
drvPadUdpCommHWTime();

DrvPadUdpCommHWTime
drvPadUdpCommHWBaseTime();

extern volatile DrvPadUdpCommHWTime drvPadUdpCommHWTimeBaseline[MAX_BPM];
/* drvPadUdpComm uses the timebase counter value (PPC, i386) for a
 * 'transaction-ID'. Thus, a time baseline can be established when
 * the last 'START all' command was sent. 
 * Usually, we send a START command at every fiducial so that we
 * can measure stuff with respect to 'fiducial'-time.
 */
extern volatile DrvPadUdpCommHWTime drvPadUdpCommPktTimeBaseline;

#ifdef __cplusplus
}
#endif

#endif
