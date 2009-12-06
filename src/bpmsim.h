#ifndef BPMSIM_H
#define BPMSIM_H

#ifdef __cplusplus
extern "C" {
#endif

unsigned
iir2_bpmsim(int16_t *pf, int nloops, int ini, int ini2, unsigned long *pn, int swp, int stride);

/* Crude algorithm from random(3). This is uniformely distributed noise
 * but becomes approximately gaussian after filtering...
 */ 
static __inline__ void
randstep(unsigned long *pn)
{
 *(pn) = *(pn) * 1103515245 + 12345;
}

/* random(3) does ((*pn) >> 16) & 0x7fff after a randstep to generate
 * a number between 0..MAXRAND
 */

static __inline__ unsigned
randval(unsigned long v)
{
	return ((v)>>16) & 0x7fff;
}

#define RANDVALMAX 32767

#ifdef __cplusplus
};
#endif

#endif
