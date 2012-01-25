/* $Id: devWfRawSig.c,v 1.3 2011/06/07 03:24:34 strauman Exp $ */

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
#include <cantProceed.h>
#include <errlog.h>

#include "wavBuf.h"
#ifndef WITHOUT_UDPCOMM_IOINT
#include <drvPadUdpComm.h>
#endif


#define DEVSUPNAM	"devWfRawSig"

#define DEBUG_PROC	1

#define SEG_RE      0
#define SEG_IM      1

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


/* Copy one segment buffer; This could be useful if ever some sort of 'piplined'
 * processing is implemented (copy segments/fragments into BPTR as packets arrive)
 */
static void
wbCpySeg(waveformRecord *waveform_ps, WavBuf wb, int sz, int n_tot, int s) __attribute__((unused));

static void
wbCpySeg(waveformRecord *waveform_ps, WavBuf wb, int sz, int n_tot, int s)
{
int      i,j,lastj,iinc, jinc, ilim, rowsz, jlim, dinc, n_frag;
uint8_t  *d_p, *s_p;

	/* If the segment describes a vector then our job is easy */
	if ( wb->segs.n == 1 || wb->segs.m == 1 ) {
		n_frag = (1 == wb->segs.m ? wb->segs.n : wb->segs.m);

		i     = s * n_frag;
		rowsz = n_frag;

		if ( i + rowsz > n_tot ) {
			rowsz = n_tot - i;
			if ( rowsz <= 0 )
				return;
		}

		memcpy( waveform_ps->bptr + i*sz, wb->segs.data[s], rowsz * sz );
		return;	
	}

	n_frag = wb->segs.n;

	d_p = waveform_ps->bptr + s * n_frag * sz;
	s_p = wb->segs.data[s];

	ilim  = (n_tot - (s+1)*n_frag)/wb->n + 1;

	if ( wb->flags & WavBufFlagCM ) {
		iinc  = sz;
		jinc  = sz * (wb->stride > wb->segs.m ? wb->stride : wb->segs.m);
		rowsz = sz;
		jlim  = jinc * wb->segs.n;
	} else {
		iinc  = sz * (wb->stride > wb->segs.n ? wb->stride : wb->segs.n);
		rowsz = sz * wb->segs.n;
		jinc  = rowsz;
		jlim  = sz * wb->segs.n;
	}

	dinc  = (wb->n - n_frag) * sz;

	for ( i = 0; i < ilim*iinc; i += iinc ) {
		for ( j = 0; j < jlim; j += jinc ) {
			memcpy( d_p, s_p + i + j, rowsz );
			d_p += rowsz;
		}
		d_p += dinc;
	}

	lastj = n_tot - ilim * wb->n - s * n_frag;

	if ( lastj > 0 ) {

		if ( wb->flags & WavBufFlagCM ) {
			jlim = lastj * jinc;
		} else {
			jlim = lastj * sz;
		}

		if ( rowsz > lastj * sz )
			rowsz = lastj * sz;

		for ( j=0; j < jlim; j += jinc ) {
			memcpy( d_p, s_p + i + j, rowsz );
			d_p += rowsz;
		}
	}
}

static void
wbCpyGeneric(waveformRecord *waveform_ps, WavBuf wb, int sz, int n_tot)
{
uint8_t  *d_p, *s_p;
int      i,j,s,n,iinc,jinc,jlim,rowsz;

	d_p = waveform_ps->bptr;
	n   = n_tot;

#if 1
	/* Optimize if segments are only vectors */
	if ( wb->segs.m == 1 || wb->segs.n == 1 ) {
		i = wb->segs.m == 1 ? wb->segs.n : wb->segs.m;
		iinc  = sz * wb->n; /* stride is ignored since we have only one vector */
		jinc  = sz * i;
		rowsz = sz * i;
		jlim  = jinc;
	} else {
		/* 'normal case' */
		if ( wb->flags & WavBufFlagCM ) {
			iinc  = sz;
			jinc  = sz * (wb->stride > wb->segs.m ? wb->stride : wb->segs.m);
			rowsz = sz;
			jlim  = jinc * wb->segs.n;
		} else {
			iinc  = sz * (wb->stride > wb->n ? wb->stride : wb->n);
			jinc  = sz * wb->segs.n;
			rowsz = sz * wb->segs.n;
			jlim  = jinc;
		}
	}
	s     = 0;
	i     = 0;
	j     = 0;
	n    *= sz;
	s_p   = wb->segs.data[s];
	do {
		while ( n >= rowsz ) {
			if ( s_p )
				memcpy( d_p, s_p + i + j, rowsz );
			else
				memset( d_p, 0,           rowsz );
			d_p += rowsz;
			n   -= rowsz;
			j   += jinc;
			if ( j >= jlim ) {
				j = 0;
				if ( ++s >= wb->segs.nsegs ) {
					/* next row */
					s  = 0;
					i += iinc;
				}
				s_p = wb->segs.data[s];
			}
		}
		rowsz = n;
	} while ( n > 0 );

#else
	/* This is less efficient; leave here for testing */
	for ( s=0; s<wb->segs.nsegs; s++ )
		wbCpySeg(waveform_ps, wb, sz, n_tot, s);
#endif

}

static void
wbCpyCplxA(waveformRecord *waveform_ps, WavBuf wb, int n_tot, double *pMin, double *pMax)
{
int i,j,n,s,jlim,iinc,jinc;
float *re_p;
float *im_p;
float *dt_p;
double min = *pMin, max = *pMax;


	if ( wb->flags & WavBufFlagCM ) {
		/* transpose */
		iinc = 1;
		jinc = wb->stride > wb->segs.m ? wb->stride : wb->segs.m;
		jlim = jinc * wb->segs.n;
	} else {
		iinc = wb->stride > wb->n ? wb->stride : wb->n;
		jinc = 1;
		jlim = wb->segs.n;
	}

	s = i = j = 0;

	re_p  = wb->segs.data[s + SEG_RE];
	im_p  = wb->segs.data[s + SEG_IM];

	dt_p = waveform_ps->bptr;
	n    = n_tot;
	while ( n > 0 ) {
		*dt_p = hypotf(re_p[i+j],im_p[i+j]);

		if ( *dt_p > max )
			max = *dt_p;
		else if ( *dt_p < max )
			min = *dt_p;
		dt_p++;
		n--;
		j += jinc;
		if ( j >= jlim ) {
			j  = 0;
			s += 2;
			if ( s  >= wb->segs.nsegs ) {
				/* next row */
				s = 0;
				i += iinc;
			}
			re_p  = wb->segs.data[s + SEG_RE];
			im_p  = wb->segs.data[s + SEG_IM];
		}
	}

	*pMin = min;
	*pMax = max;
}

static void
wbCpyCplxP(waveformRecord *waveform_ps, WavBuf wb, int n_tot, double *pMin, double *pMax)
{
int i,j,s,n,jlim,iinc,jinc;

float *re_p;
float *im_p;
float *dt_p;
double phas;
float  re_o, im_o;
float  re,   im;
double min = *pMin, max = *pMax;


	if ( wb->flags & WavBufFlagCM ) {
		/* transpose */
		iinc = 1;
		jinc = wb->stride > wb->segs.m ? wb->stride : wb->segs.m;
		jlim = jinc * wb->n;
	} else {
		iinc = wb->stride > wb->n ? wb->stride : wb->n;
		jinc = 1;
		jlim = wb->segs.n;
	}

	n    = n_tot;
	dt_p = waveform_ps->bptr;
	i = j = s = 0;

	re_p  = wb->segs.data[s + SEG_RE];
	im_p  = wb->segs.data[s + SEG_IM];

	phas = 0.;
	re_o = 1.;
	im_o = 0.;

	while ( n > 0 ) {

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
		n--;
		j += jinc;

		if ( j >= jlim ) {
			j  = 0;
			s += 2;
			if ( s  >= wb->segs.nsegs ) {
				/* next row */
				s = 0;
				i += iinc;

				phas = 0.;
				re_o = 1.;
				im_o = 0.;
			}
			re_p  = wb->segs.data[s + SEG_RE];
			im_p  = wb->segs.data[s + SEG_IM];
		}
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

	n = wb->n * wb->m;

	if ( n > waveform_ps->nelm )
		n = waveform_ps->nelm;
	if ( n != waveform_ps->nord ) {
		waveform_ps->nord = n;
		post |= 1;
	}

	/* Fill in missing segment size as a courtesy */
	if ( 0 == wb->segs.m ) {
		if ( 1 == wb->segs.nsegs || (2 == wb->segs.nsegs && WavBufCplx == wb->type) )
			wb->segs.m = wb->m;
		else
			cantProceed("wavBuf: Missing segment dimension 'm'\n");
	}

	if ( 0 == wb->segs.m ) {
		if ( 1 == wb->segs.nsegs || (2 == wb->segs.nsegs && WavBufCplx == wb->type) )
			wb->segs.n = wb->n;
		else
			cantProceed("wavBuf: Missing segment dimension 'n'\n");
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
