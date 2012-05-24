/* $Id: drvPadAdcStream.h,v 1.1 2011/05/02 17:47:46 strauman Exp $ */
#ifndef DRV_PAD_ADC_STREAM_H
#define DRV_PAD_ADC_STREAM_H

#include <padProto.h>
#include <padStream.h>

/* read PAD FIFO into column or row-major array.
 *
 * 'notFirst': If routine is called multiple
 *        times to read a single waveform then 'notFirst'
 *        must be 0 the first time and nonzero thereafter.
 *        This is because the FIFO needs a few initial
 *        clock cycles...
 *
 * 'n':   number of samples *per-channel* to read
 *
 * 'channels': bitmask of channels to read. NOTE: col-major
 *        currently only supports reading all 4 channels,
 *        i.e., channels == 0x0f.
 *        E.g., to read channel #2 in row-major format
 *        pass channels = (1<<2).
 *
 * 'le' - if nonzero causes bytes to be swapped so that the data
 * are converted to little-endian byteorder. This feature
 * and the 'volatile' declaration of the destination address
 * are added so that we can directly write to the LAN9118 chip's
 * TX fifo. This violates nice encapsulation but gains ~30% / 80us
 * in speed.
 *
 * 'n' is the number of 
 */

void
drvPadReadFifosColMajor(volatile void *dst_p_v, int notFirst, int channels, int n, int le);

void
drvPadReadFifosRowMajor(volatile void *dst_pv, int notFirst, int ch, int n, int le);

/* Getdata routine to be used with padStreamSend */
void *
drvPadAdcStream_getdata(void *packBuffer, int idx, int channels, int nsamples_tot, int nsamples_frag, int d32, int endianLittle, int colMajor, void *uarg);

/* Argument checker to be used in combination with drvPadAdcStream_getdata() */
int
drvPadAdcStream_start_stop_cb(PadRequest req, PadStrmCommand start, void *uarg);

long 
drvPadAdcStreamReport(int level);
	
epicsThreadId
drvPadAdcStreamInit(int (*start_stop_cb)(PadRequest, PadStrmCommand, void*));

void
padUdpThread(void *arg_p);

#endif
