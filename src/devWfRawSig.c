/* $Id: devWfRawSig.c,v 1.10 2010/01/18 19:16:50 strauman Exp $ */

/*=============================================================================
 
  Name: devWfRawSig.c

  Abs:  Device support for saving raw digitizer data into a waveform 
        (4 channels into a single, row-major array).

  Auth: 10-nov-2006, Till Straumann (TSS)
  Rev:

-----------------------------------------------------------------------------*/

#if 0
#include "copyright_SLAC.h"	/* SLAC copyright comments */
#endif
 
/*-----------------------------------------------------------------------------
 
  Mod:  (see CVS log)

 
=============================================================================*/

#ifdef __STRICT_ANSI__
#undef __STRICT_ANSI__
#endif

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <epicsExport.h>
#include <devSup.h>
#include <waveformRecord.h>
#include <menuFtype.h>
#include <alarm.h>
#include <dbEvent.h>
#include <dbAccess.h>
#include <recGbl.h>

#include "wavBuf.h"
#ifndef WITHOUT_UDPCOMM_IOINT
#include <drvPadUdpComm.h>
#endif


#define DEVSUPNAM	"devWfRawSig"

#define DEBUG_PROC	1

int    devWfRawSigDebug = 0;

#define FLG_REG_WITH_WAVBUF	(1<<0)
#define FLG_SET_LOPR        (1<<1)
#define FLG_SET_HOPR        (1<<2)

typedef struct DevWfRawSigDpvt_ts_ {
#ifndef WITHOUT_UDPCOMM_IOINT
	struct DrvPadUdpCommDpvtRec_ drvPadUdpCommPart;
#endif
	unsigned                     flags;
	void                         *wbpvt;
} DevWfRawSigDpvt_ts, *DevWfRawSigDpvt_tps;

/* Forward Declarations */

static long
report(int interest)
{
	epicsPrintf(DEVSUPNAM": raw BPM digitizer data into a single waveform\n");
	return 0;
}

static long
init(int after)
{
	return 0;
}

static long
init_record(void *record_p)
{
waveformRecord      *waveform_ps = record_p;
DevWfRawSigDpvt_tps	dpvt_ps;

	/* Check DB configuration */

	if ( VME_IO != waveform_ps->inp.type ) {
		epicsPrintf(DEVSUPNAM": Error -- INP is not VME_IO (but type %i)\n",
					waveform_ps->inp.type);
		goto egress;
	}

	if ( !(dpvt_ps = calloc(1, sizeof(*dpvt_ps))) ) {
		epicsPrintf(DEVSUPNAM": Error -- no memory\n");
		goto egress;
	}
	waveform_ps->dpvt = dpvt_ps;

	dpvt_ps->flags    = 0;

	if ( strstr(waveform_ps->inp.value.vmeio.parm,"wbscan") ) {
		dpvt_ps->flags |= FLG_REG_WITH_WAVBUF;
		wavBufRegisterRecord(
			record_p,
			waveform_ps->inp.value.vmeio.card,
			waveform_ps->inp.value.vmeio.signal
		);
	}

	/* If HOPR/LOPR are not set initially we maintain them dynamically */
	if ( 0. == waveform_ps->hopr )
		dpvt_ps->flags |= FLG_SET_HOPR;

	if ( 0. == waveform_ps->lopr )
		dpvt_ps->flags |= FLG_SET_LOPR;

#ifndef WITHOUT_UDPCOMM_IOINT
	if ( drvPadUdpCommDpvtInit(&dpvt_ps->drvPadUdpCommPart, waveform_ps->inp.value.vmeio.card) )
		goto egress;
#endif

	return 0;

egress:
	waveform_ps->pact = 1;
	return -1;
}

#ifndef WITHOUT_UDPCOMM_IOINT
static long
get_ioint_info(int cmd, struct dbCommon *record_ps, IOSCANPVT *pvt_ps)
{
waveformRecord      *waveform_ps = (waveformRecord*)record_ps;
DevWfRawSigDpvt_tps dpvt_ps      = waveform_ps->dpvt;

	if ( (dpvt_ps->flags & FLG_REG_WITH_WAVBUF) ) {
		/* if we are added to the list cancel our subscription with
		 * wavBuf; otherwise renew it.
		 */
		wavBufRegisterRecord(
			cmd ? 0 : record_ps,
			waveform_ps->inp.value.vmeio.card,
			waveform_ps->inp.value.vmeio.signal
		);
	}

	return drvPadUdpCommGetIointInfo(cmd, record_ps, pvt_ps);
}
#endif

static void
wbCpyGeneric(waveformRecord *waveform_ps, WavBuf wb, int sz, int n)
{
uint8_t *d_p, *s_p;
int     i,j,mnsz,msz,ilim;

	d_p = waveform_ps->bptr;
	s_p = wb->data;

	/* deal with the case where destination < source area */
	ilim = n/wb->n;

	if ( wb->flags & WavBufFlagCM ) {
		/* transpose */

		msz  = ( wb->stride > wb->m ? wb->stride : wb->m ) * sz;
		mnsz = wb->n * msz;

		for ( i = 0; i < ilim*sz; i+=sz ) {
			for ( j = 0; j< mnsz; j+=msz ) {
				memcpy(d_p, s_p + i + j, sz);
				d_p += sz;
			}
		}
		for ( j = 0; j< (n-ilim*wb->n)*msz; j+=msz ) {
			memcpy(d_p, s_p + i + j, sz);
			d_p += sz;
		}
	} else {
		if ( wb->stride > wb->n ) {
			for ( i=0; i< ilim; i++ ) {
				memcpy(d_p, s_p, wb->n * sz);
				d_p += wb->n * sz; 
				s_p += wb->stride * sz;
			}
			/* memcpy should handle the case where nothing is to be
			 * copied...
			 */
			memcpy(d_p, s_p, (n - ilim*wb->n)*sz);
		} else {
			memcpy(d_p, s_p, n * sz);
		}
	}
}

static void
wbCpyCplxA(waveformRecord *waveform_ps, WavBuf wb, int n, double *pMin, double *pMax)
{
int i,j,ilim,iinc,jinc;

float *re_p = wb->data;
float *im_p = wb->data1;
float *dt_p = waveform_ps->bptr;
double min = *pMin, max = *pMax;

	if ( wb->flags & WavBufFlagCM ) {
		/* transpose */
		iinc = 1;
		jinc = wb->stride > wb->m ? wb->stride : wb->m;
	} else {
		iinc = wb->stride > wb->n ? wb->stride : wb->n;
		jinc = 1;
	}

	/* deal with the case where destination < source area */
	ilim = n/wb->n;

	for ( i = 0; i < ilim*iinc; i+=iinc ) {
		for ( j = 0; j< wb->n * jinc; j+=jinc ) {
			*dt_p = hypotf(re_p[i+j],im_p[i+j]);
			if ( *dt_p > max )
				max = *dt_p;
			else if ( *dt_p < max )
				min = *dt_p;
			dt_p++;
		}
	}
	for ( j = 0; j< (n-ilim*wb->n)*jinc; j+=jinc ) {
		*dt_p = hypotf(re_p[i+j],im_p[i+j]);
		if ( *dt_p > max )
			max = *dt_p;
		else if ( *dt_p < max )
			min = *dt_p;
		dt_p++;
	}
	*pMin = min;
	*pMax = max;
}

static void
wbCpyCplxP(waveformRecord *waveform_ps, WavBuf wb, int n, double *pMin, double *pMax)
{
int i,j,ilim,iinc,jinc;

float *re_p = wb->data;
float *im_p = wb->data1;
float *dt_p = waveform_ps->bptr;
double phas;
float  re_o, im_o;
float  re,   im;
double min = *pMin, max = *pMax;

	if ( wb->flags & WavBufFlagCM ) {
		/* transpose */
		iinc = 1;
		jinc = wb->stride > wb->m ? wb->stride : wb->m;
	} else {
		iinc = wb->stride > wb->n ? wb->stride : wb->n;
		jinc = 1;
	}

	/* deal with the case where destination < source area */
	ilim = n/wb->n;
printf("ilim %i, n %i, wb->n %i\n",ilim,n,wb->n);

	for ( i = 0; i < ilim*iinc; i+=iinc ) {
		phas = 0.;
		re_o = 1.;
		im_o = 0.;
		for ( j = 0; j< wb->n * jinc; j+=jinc ) {
if ( 1*iinc == i && j < 4*jinc ) {
	printf("re %f, im %f, phas %g ",re_p[i+j], im_p[i+j], phas);
}
	printf("%i %i %i %i\n",i,j,iinc,jinc);
			re = re_p[i+j]; im = im_p[i+j];
			if ( 0. == re && 0. == im )
				re = 1.;
			phas   += atan2f(im*re_o - im_o*re, re*re_o + im*im_o);
if ( 1*iinc == i && j < 4*jinc ) {
	printf("re %f, im %f, phas %g\n",re_p[i+j], im_p[i+j], phas);
}
			re_o    = re;
			im_o    = im;
			if ( phas > max )
				max = phas;
			else if ( phas < min )
				min = phas;
			*dt_p++ = phas;
		}
	}

	phas = 0.;
	re_o = 1.;
	im_o = 0.;
	for ( j = 0; j< (n-ilim*wb->n)*jinc; j+=jinc ) {
		re = re_p[i+j]; im = im_p[i+j];
		if ( 0. == re && 0. == im )
			re = 1.;
		phas   += atan2f(im*re_o - im_o*re, re*re_o + im*im_o);
		re_o    = re;
		im_o    = im;
		if ( phas > max )
			max = phas;
		else if ( phas < min )
			min = phas;
		*dt_p++ = phas;
	}
	*pMin = min;
	*pMax = max;
}

static long
read_waveform(void *record_p)
{
waveformRecord   *waveform_ps  = record_p;
DevWfRawSigDpvt_tps dpvt_ps    = waveform_ps->dpvt;
WavBuf           wb            = 0;
long             rval          = 0;
int              post          = 0;
int              n,sz,i;
struct vmeio	 *l            = &waveform_ps->inp.value.vmeio;
int              need_ts       = 0;
double           min           = 1.E23;
double           max           =-1.E23;
int32_t          imin, imax;

	if ( devWfRawSigDebug & DEBUG_PROC )
			epicsPrintf("Entering READ\n");

	if ( waveform_ps->pact ) {
		/* completion phase of async processing */
		wb = wavBufAsyncComplete(record_p, &dpvt_ps->wbpvt, l->card, l->signal);
		if ( ! wb ) {
			rval = -1;
			goto bail;
		}
	} else {
		if ( !(wb = wavBufGet(l->card, l->signal)) ) {
			/* Hmm - what to do here? 
             * if we return an error because this
             * record is scanned before new data is
             * available. (May happen in simulation setup
             * w/o calibration data: cal data times out
             * and processes the diagnostic scanlist
             * when no new stripline data is available
             * yet.)
             * The user would see 'glitches' (waveforms
             * going away).
             *
             * OTOH if we return 'OK (0)' then we
             * just keep old data. This means that 
             * waveforms never go away even if we 
             * have a real timeout.
             *
             * Given that the first scenario should only
             * happen in the simulator environment
             * we prefer it over the second alternative.
             */
			rval = -1;
			goto bail;
		}
		/* is there asynchronous processing for this
		 * record implemented ?
		 */
		if ( 0 == wavBufAsyncStart(record_p, &dpvt_ps->wbpvt, l->card, l->signal, wb) ) {
			/* yes; mark PACT */
			waveform_ps->pact = 1;
			return 0;
		}
	}

	if ( epicsTimeEventDeviceTime == waveform_ps->tse ) {
		if ( wb->ts.secPastEpoch )
			waveform_ps->time = wb->ts;
		else
			need_ts = 1;
	}

	/* type cross-check */
	switch ( wb->type ) {
		default:
			epicsPrintf(DEVSUPNAM":(%s) Error -- WavBuf type (0x%x) not supported\n", waveform_ps->name, wb->type);
			rval = -1;
		goto bail;
		
		case WavBufInt16:
			if ( waveform_ps->ftvl != menuFtypeSHORT )
				rval = -1;
		break;

		case WavBufInt32:
			if ( waveform_ps->ftvl != menuFtypeLONG )
				rval = -1;
		break;

		case WavBufFloat:
		case WavBufCplx:
			if ( waveform_ps->ftvl != menuFtypeFLOAT )
				rval = -1;
		break;
	}

	if ( (sz=wavBufTypeSize(wb->type)) != dbValueSize(waveform_ps->ftvl) ) {
		epicsPrintf(DEVSUPNAM"(%s): Type size mismatch! Waveform: %i, FTVL: %li\n", waveform_ps->name, sz, dbValueSize(waveform_ps->ftvl));
		rval = -1;
		goto bail;
	}

	n = wb->n * wb->m;;
	if ( n > waveform_ps->nelm )
		n = waveform_ps->nelm;
	if ( n != waveform_ps->nord ) {
		waveform_ps->nord = n;
		post |= 1;
	}

	switch ( wb->type ) {
		case WavBufCplx:
			/* Abuse RARM to switch between amplitude and phase */
			if ( waveform_ps->rarm >= 0 ) {
				wbCpyCplxA(waveform_ps, wb, n, &min, &max);
			} else {
				wbCpyCplxP(waveform_ps, wb, n, &min, &max);
			}
		break;

		default:
			wbCpyGeneric(waveform_ps, wb, sz, n);
			if ( (dpvt_ps->flags & (FLG_SET_HOPR | FLG_SET_LOPR)) ) {
				switch ( wb->type ) {
					case WavBufInt16:
						imin = 32767; imax = -32768;
						for ( i=0; i<n; i++ ) {
							int16_t v = ((int16_t*)waveform_ps->bptr)[i];
							if ( v > imax ) imax = v; else if ( v < imin ) imin = v;
						}
						min = (double)imin; max = (double)imax;
						break;

					case WavBufInt32:
						imin = 0x7fffffff;
						imax = 0x80000000;
						for ( i=0; i<n; i++ ) {
							int32_t v = ((int32_t*)waveform_ps->bptr)[i];
							if ( v > imax ) imax = v; else if ( v < imin ) imin = v;
						}
						min = (double)imin; max = (double)imax;
						break;

					case WavBufFloat:
						for ( i=0; i<n; i++ ) {
							float v = ((float*)waveform_ps->bptr)[i];
							if ( v > max ) max = v; else if ( v < min ) min = v;
						}
						break;

				}
			}
		break;
	}

	waveform_ps->udf = FALSE;

bail:

	if ( rval ) {
		recGblSetSevr(waveform_ps, READ_ALARM, INVALID_ALARM);
		need_ts = (epicsTimeEventDeviceTime == waveform_ps->tse);
		if ( waveform_ps->nord > 0 ) {
			waveform_ps->nord = 0;
			post |= 1;
		}
		max = min = 0.;
	}

	if ( (dpvt_ps->flags & FLG_SET_HOPR) ) {
		waveform_ps->hopr = max;
	}
	if ( (dpvt_ps->flags & FLG_SET_LOPR) ) {
		waveform_ps->lopr = min;
	}

	/* release data object */
	wavBufFree(wb);

	if ( need_ts )
		epicsTimeGetCurrent(&waveform_ps->time);

	if ( post & 1 )
	    db_post_events(waveform_ps, &waveform_ps->nord, (DBE_VALUE|DBE_LOG));
	if ( post & 2 )
	    db_post_events(waveform_ps, &waveform_ps->rarm, (DBE_VALUE|DBE_LOG));

	if ( devWfRawSigDebug & DEBUG_PROC )
			epicsPrintf("Leaving READ\n");
	return rval;
}

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read;
} devWfRawSig = {
	5,
	report,
	init,
	init_record,
#ifndef WITHOUT_UDPCOMM_IOINT
	get_ioint_info,
#else
	0,
#endif
	read_waveform
};

epicsExportAddress(dset, devWfRawSig);
