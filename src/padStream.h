#ifndef PAD_STREAM_H
#define PAD_STREAM_H
/* $Id: padStream.h,v 1.7 2011/04/25 21:16:47 strauman Exp $ */

#include <padProto.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Data stream implementation. This could all be done over the
 * udpSock abstraction but less efficiently since we would have
 * to copy the PAD fifo to memory first instead of copying the
 * PAD fifo to the lan9118 chip directly...
 */

/* Setup everything for stream processing.
 * The 'cb' callback is executed (padUdpHandler task context)
 * with a non-NULL 'scmd' argument when the stream is
 * started and again with a NULL argument when the stream
 * is stopped (enable/disable data source).
 * The callback should return 0 on success, nonzero if the
 * stream cannot be started or stopped.
 */
typedef int (*PadStreamStartStopCB)(PadRequest req, PadStrmCommand scmd, void *uarg);

int
padStreamInitialize(void *if_p, PadStreamStartStopCB cb, void *uarg);

/* cleanup (for module unloading) */
int
padStreamCleanup();

/* refresh timestamp and transaction id - 
 * check for valid peer IP.
 */
int
padStreamPet(PadRequest req, uint32_t hostip);

/* Function that actually starts streaming transfer
 * to a host.
 *        'me': our channel #
 *    'hostip': IP address in network byte order.
 */
int
padStreamStart(PadRequest req, PadStrmCommand scmd, int me, uint32_t hostip);

#define PADSTRM_GETDATA_F_LE	(1<<0)
#define PADSTRM_GETDATA_F_CM	(1<<1)
#define PADSTRM_GETDATA_F_32	(1<<2)

typedef void * (*PadStreamGetdataProc)(void *packBuffer, int idx, int channels, int nsamples, int d32, int endianLittle, int colMajor, void *uarg);

int
padStreamSend(PadStreamGetdataProc getdata, int type, void *uarg);

/* execute 'padStreamSend' with test data */
int
padStreamTest(int type);

/* execute 'padStreamSend' with simulated/generated data
 * if 'scmd' is non-NULL then the simulation parameters
 * are updated from 'scmd'. If 'hostip' is nonzero then
 * 'scmd' is only accepted if 'hostip' matches the peer
 * who had started the stream. Otherwise an error is
 * (-EADDRINUSE) is returned.
 * 
 * RETURNS: zero on success, nonzero on error (-errno).
 */
int
padStreamSim(PadSimCommand scmd, uint32_t hostip);

typedef struct PadStripSimValRec_ {
	int32_t	val[PADRPLY_STRM_NCHANNELS];
} PadStripSimValRec, *PadStripSimVal;

/* 'getdata' callback for generating simulated waveforms
 * Passing a delta function plus noise through a 2nd order
 * bandpass filter.
 * The amplitudes of the delta functions are passed in
 * 'uarg' which must be a 'PadStripSimVal' pointer.
 */

void *
padStreamSim_iir2_getdata(void *packetBuffer,
			int idx,
			int channels,
			int nsamples,
			int d32,
			int little_endian,
			int column_major,
			void *uarg);

/* Function pointer you can tweak to install your
 * own simulation routine (called by padStreamSim()
 * when a PADCMD_SIM is executed)
 * Defaults to padStreamSim_iir2_getdata.
 */
extern PadStreamGetdataProc padStreamSim_getdata;


/* Function that actually stops streaming transfer
 * - check for valid peer IP.
 *
 * NOTE: if 'hostip' == 0 then the check for a valid peer
 *       is skipped and the stream forcefully stopped.
 */
int
padStreamStop(uint32_t hostip);

typedef enum {
	PadDataBpm = 0,
	PadDataCalRed,
	PadDataCalGrn,
	PadDataCavity
} PadDataKind;

#ifdef __cplusplus
}
#endif

#endif
