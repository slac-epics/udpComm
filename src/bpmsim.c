/* Simulate BPM/AFE response with a crude 2nd order iir filter */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "hwtmr.h"
#include "bpmsim.h"

/* coldfire uc5282 has no FPU */
typedef int FiltNumber;

/* coefficients are shifted 16 bits left; signals
 * are 16 bit.
 */
#define FNUM(x)  ((FiltNumber)((float)(1<<16)*(x)))

/* renormalize after multiplication by coefficients */
#define FNORM(x) ((x)>>16)

#define NOISESTEP(pn) randstep(pn)

/* How many bits of noise (in-band) to use */
#define IBNOISEBITS 6

/* State of the IIR filter */
struct Iir2Stat_ {
	FiltNumber y[2];
	FiltNumber x[2];
	int        i;
};

/* One step of bandpass filtering.
 * The response function is decomposed in partial fractions
 * and specialized to the case where the odd parts are of
 * opposite sign:
 */

#define IIRBP(y1a,y2a,y1b,y2b,x0,x1,x2,cn0,cn1,cn2,cd1,cd2) \
	do {	\
		FiltNumber tmp1 = (cn0)*(x0)+(cn2)*(x2); \
		FiltNumber tmp2 = (cn1)*(x1);            \
		y2a = FNORM(tmp1 + tmp2 - (cd1)*(y1a) - (cd2)*(y2a)); \
		y2b = FNORM(tmp1 - tmp2 + (cd1)*(y1b) - (cd2)*(y2b)); \
		x2 = x0; \
	} while (0)

/* Do two steps of the above filtering (wraps state around once) */
#define IIRBP2(ya,yb,x,x0,x1,cn0,cn1,cn2,cd1,cd2) \
	do {	\
		IIRBP(ya[1],ya[0],yb[1],yb[0],x0,x[1],x[0],cn0,cn1,cn2,cd1,cd2);	\
		IIRBP(ya[0],ya[1],yb[0],yb[1],x1,x[0],x[1],cn0,cn1,cn2,cd1,cd2);	\
	} while (0)

/* Coefficients for a 2nd order modified chebyshev bandpass fc = .25*fs, BW = 4/100 * fs, ripple .05
 *
 *                        2    
 *       0.0200136 ( 1 - z )    
 *       ----------------------------   
 *                              2   4  
 *       0.6908829 + 1.6066152 z + z   
 *
 * Partial fraction decomposition yields the coefficients below...
 * 
 */                                                                      
#define CN0 FNUM(0.)
#define CN1 FNUM(0.0933508)
#define CN2 FNUM(0.0120391)

#define CD1 FNUM(0.2361611)
#define CD2 FNUM(0.8311936)

static __inline__ int16_t
bswap(int16_t x)
{
	return (x<<8) | (((uint16_t)x)>>8);
}

/* simulate PAD overflow signalization (LSB == OVR) */
static __inline__ int16_t lmt(int v)
{
int16_t rval;
	rval = v & 0x0000fffe;

	if ( v > 32767 || v < -32768 ) rval |= 1;
	return rval;
}

unsigned
iir2_bpmsim(int16_t *pf, int nloops, int ini, int ini2, unsigned long *pn, int swp, int stride)
{
unsigned then = Read_hwtimer();
FiltNumber ysa[2] = { FNUM(0.), FNUM(0.) }, xsa[2] = { FNUM(0.), FNUM(0.)};
FiltNumber ysb[2] = { FNUM(0.), FNUM(0.) };
signed char n;
int16_t     *pf1 = pf+stride;
int         ini1 = ini;

	NOISESTEP(pn);
	/* use top 4 bits of noise */
	ini1 += ((int)*pn)>>(32-IBNOISEBITS);
	n     = (signed char)(*pn)>>24;
	/* This is 'in-band' noise. We should also add wide-band noise
	 * at the output but we probably wouldn't have time to make it gaussian
	 */
	IIRBP2(ysa,ysb,xsa,(FiltNumber)ini1,(FiltNumber)(ini2+(n>>(8-IBNOISEBITS))),CN0,CN1,CN2,CD1,CD2);
	*pf = lmt(ysa[0] + ysb[0]); *pf1 = lmt(ysa[1] + ysb[1]);
	if ( swp ) {
		*pf = bswap(lmt(ysa[0] + ysb[0])); *pf1 = bswap(lmt(ysa[1] + ysb[1]));
	} else {
		*pf = lmt(ysa[0] + ysb[0]);      *pf1 = lmt(ysa[1] + ysb[1]);
	}
	pf  = pf1 + stride;
	pf1 = pf  + stride;
	nloops-=2;

	if ( 0 ) {
	ini1  = -ini;
	NOISESTEP(pn);
	/* use top 4 bits of noise */
	ini1 += ((int)*pn)>>(32-IBNOISEBITS);
	n     = (signed char)(*pn)>>24;
	IIRBP2(ysa,ysb,xsa,(FiltNumber)ini1,(FiltNumber)0,CN0,CN1,CN2,CD1,CD2);
	*pf = lmt(ysa[0] + ysb[0]); *pf1 = lmt(ysa[1] + ysb[1]);
	if ( swp ) {
		*pf = bswap(lmt(ysa[0] + ysb[0])); *pf1 = bswap(lmt(ysa[1] + ysb[1]));
	} else {
		*pf = lmt(ysa[0] + ysb[0]);      *pf1 = lmt(ysa[1] + ysb[1]);
	}
	pf  = pf1 + stride;
	pf1 = pf  + stride;
	nloops-=2;
	}

	if (swp) {
		while ( nloops > 0 ) {
			NOISESTEP(pn);
			ini = ((int)*pn)>>(32-IBNOISEBITS);
			n   = (signed char)(*pn)>>24;
			IIRBP2(ysa,ysb,xsa,(FiltNumber)ini,n>>(8-IBNOISEBITS),CN0,CN1,CN2,CD1,CD2);
			*pf  = bswap(lmt(ysa[0] + ysb[0]));
			*pf1 = bswap(lmt(ysa[1] + ysb[1]));
			pf  = pf1 + stride;
			pf1 = pf  + stride;
			nloops-=2;
		}
	} else {
		while ( nloops > 0 ) {
			NOISESTEP(pn);
			ini = ((int)*pn)>>(32-IBNOISEBITS);
			n   = (signed char)(*pn)>>24;
			IIRBP2(ysa,ysb,xsa,(FiltNumber)ini,n>>(8-IBNOISEBITS),CN0,CN1,CN2,CD1,CD2);
			*pf  = lmt(ysa[0] + ysb[0]);
			*pf1 = lmt(ysa[1] + ysb[1]);
			pf  = pf1 + stride;
			pf1 = pf  + stride;
			nloops-=2;
		}
	}
	return Read_hwtimer() - then;
}

#ifdef MAIN

#include  <math.h>

int
main(int argc, char **argv)
{
int              i,i2;
int16_t buf[128];
int     N = sizeof(buf)/sizeof(buf[0]);
float   e,x;
unsigned long nois = 1;

	if ( argc < 2 || 1!=sscanf(argv[1],"%i",&i) )
		i = 10000;
	if ( argc < 3 || 1!=sscanf(argv[2],"%i",&i2) )
		i2 = 10000;
	iir2_bpmsim(buf, sizeof(buf)/sizeof(buf[0]), i, i2, &nois, 0, 1);

	e = 0.;
	for ( i=0; i<N; i++) {
		x  = (float)buf[i];
		e += x*x;
		printf("%f\n",(float)buf[i]);
	}
	printf("Energy: %f, RMS %f\n", e, sqrtf(e/(float)N));
}
#endif
