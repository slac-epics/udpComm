#include <SDDS.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "sddsrd.h"

#define NCOLS   4

#ifdef __SSE2__
#define VECTSZ 16
#endif

#ifdef VECTSZ

#define VECTALGN(x)   ((((uintptr_t)(x)) + (VECTSZ) - 1) & ~((VECTSZ)-1))
typedef int16_t vShort    __attribute__((vector_size(VECTSZ),may_alias));
typedef vShort  vShort_a  __attribute__((may_alias));

#ifdef __SSE2__
static __inline__ vShort vec_swab(vShort x)
{
	return __builtin_ia32_psrlwi128(x,8) | __builtin_ia32_psllwi128(x,8);
}
#else
#error "Need to implement vec_swab() for your CPU"
#endif

#else
#define VECTALGN(x)   ((uintptr_t)(x))
#endif


#define FLG_CM (1<<0)
#define FLG_LE (1<<1)

static int bigEndian()
{
union {
	uint8_t  x[2];
	uint16_t tst;
} endian = { x : {0xBE, 0} };
	return(endian.tst == 0xBE00);
}

void
sddsDatClean(SddsFileDat wrk)
{
	if ( wrk ) {
		SddsPage pn,p;
		for ( p = wrk->pages; p; p = pn ) {
			pn = p->next;
			free(p);
		}
		free(wrk);
	}
}

static void vswab(SddsPage p)
{
int         j;
int         N = p->nSamples * NCOLS;
#ifdef VECTSZ
int nvecs     = N/(VECTSZ/sizeof(*p->data));
vShort    *pv = (vShort*)p->data;

	for ( j = 0; j<nvecs; j++ )
		pv[j] = vec_swab(pv[j]);

	j *= (VECTSZ/sizeof(*p->data));
#else
	j  = 0;
#endif
	for ( ; j<N; j++ ) {
		uint16_t tmpu;
		tmpu = p->data[j];
		p->data[j] = (int16_t)(tmpu>>8 | tmpu<<8);
	}
}

SddsFileDat
sddsFileSlurp(
	const char *fnam,
	const char *coln1,
	const char *coln2,
	const char *coln3,
	const char *coln4,
	int        pgFst,
	int        pgLst,
	int        nSamples,
	int        flags
	)
{
SDDS_DATASET sdds;
SddsFileDat  rval  = 0;
SddsPage    *tailp,tail;
SddsFileDat  wrk   = 0;
int          pp,m,n,i,ii,j,jj,sz,st;
int16_t     *cols [NCOLS] = {    0,     0,     0,     0};
const char  *colns[NCOLS] = {coln1, coln2, coln3, coln4};
long long    mean[NCOLS]  = {  0LL,   0LL,   0LL,   0LL};
int16_t      means[NCOLS];

	if ( !fnam ) {
		fprintf(stderr,"sddsFileSlurp: no filename given\n");
		return 0;
	}
	if ( ! SDDS_InitializeInput(&sdds, (char*)fnam) ) {
		fprintf(stderr,"sddsFileSlurp: SDDS initialization failed\n");
		return 0;
	}

	if ( pgFst < 0 )
		pgFst = 0;

	if ( pgFst ) {
		if ( ! SDDS_GotoPage( &sdds, pgFst ) ) {
			fprintf(stderr,"sddsFileSlurp: unable to go to page #%u\n", pgFst);
			goto bail;
		}
	}

	if ( ! (wrk = calloc(1, sizeof(wrk))) ) {
		fprintf(stderr,"sddsFileSlurp: no memory for work struct\n");
		goto bail;
	}

	tailp = &wrk->pages;

	for ( pp = pgFst; (pgLst < 0 || pp <= pgLst ) && !SDDS_CheckEndOfFile( &sdds ); pp++ ) {
		if ( 0 >= (st = SDDS_ReadPage( &sdds )) ) {
			if ( -1 == st && SDDS_ASCII == sdds.original_layout.data_mode.mode ) {
				/* When reading ASCII files, SDDS_ReadPage seems to detect
                 * EOF and return -1...
                 */
				break;
			}
			fprintf(stderr,"sddsFileSlurp: failed to read page %u\n", pp);
			goto bail;
		}
		m = SDDS_RowCount( &sdds );
		n = SDDS_ColumnCount( &sdds );

		if ( nSamples < 0 )
			nSamples = m;

		if ( m > nSamples )
			m = nSamples;

		for ( i = 0; i<sizeof(cols)/sizeof(cols[0]); i++ ) {
			if ( colns[i] ) {
				if ( ! (cols[i] = SDDS_GetColumnInShort( &sdds, (char*)colns[i] )) ) {
					fprintf(stderr,"Unable to read column '%s' (page %u)\n",
						colns[i], pp);
					goto bail;
				}
			}
		}

		sz  = sizeof(**tailp);                            /* page struct itself */
		sz += sizeof((*tailp)->data) * (nSamples*NCOLS);  /* the numbers        */
#ifdef VECTSZ
		sz += VECTSZ-1;                                   /* vector alignment   */
		sz += VECTSZ-1;                                   /* pad end for vect. access */
#endif

		/* have all columns ready */
		if ( !(tail = *tailp = calloc(1, sz)) ) {
			fprintf(stderr,"sddsFileSlurp: failed to alloc memory for page %u\n", pp);
			goto bail;
		}

		tail->data = (void*)VECTALGN(tail->buf);

		if ( flags & FLG_CM ) {
			/* Now copy the data into desired format;  */
			for ( j=jj=0; j<m; j++, jj+=NCOLS ) {
				for ( i=0; i<NCOLS; i++ ) {
					mean[i] += (tail->data[i+jj] = cols[i] ? cols[i][j] : 0);
				}
			}

			for ( i=0; i<NCOLS; i++ ) {
				means[i] = (int16_t)( mean[i] / (long long)m);
			}

			/* pad with mean value if nsamples > m */
			for ( ; j<nSamples; j++, jj+=NCOLS ) {
				for ( i=0; i<NCOLS; i++ ) {
					tail->data[i+jj] = means[i];
				}
			}

		} else {
			/* Now copy the data into desired format;  */
			for ( i=ii=0; i<NCOLS; i++, ii+=nSamples ) {
				for ( j=0; j<m; j++ ) {
					mean[i] += (tail->data[ii+j] = cols[i] ? cols[i][j] : 0);
				}
			}

			for ( i=0; i<NCOLS; i++ ) {
				means[i] = (int16_t)( mean[i] / (long long)m);
			}

			/* Now copy the data into desired format;  */
			for ( i=ii=0; i<NCOLS; i++, ii+=nSamples ) {
				for ( j=m; j<nSamples; j++ ) {
					tail->data[ii+j] = means[i];
				}
			}
		}

		tail->nSamples = nSamples;

		if ( (flags & FLG_LE) ^ (bigEndian() ? 0 : FLG_LE) ) {
			/* byte-swap necessary */
			vswab(tail);
		}

		tail->flags    = flags & (FLG_CM | FLG_LE);

		for ( i = 0; i<sizeof(cols)/sizeof(cols[0]); i++ ) {
			if ( cols[i] ) {
				SDDS_Free(cols[i]);
				cols[i] = 0;
			}
		}

		tailp = &tail->next;
	}
	
	wrk->numPages = pp - pgFst;

	rval = wrk;
	wrk  = 0;

bail:
	SDDS_Terminate( &sdds );
	for ( i=0; i<sizeof(cols)/sizeof(cols[0]); i++ ) {
		if ( cols[i] ) 
			SDDS_Free(cols[i]);
	}
	sddsDatClean(wrk);
	return rval;
}

#if 0
typedef uint32_t ua32 __attribute__((may_alias));

#define XPOSE2x2(a,b)                   \
	do {                                \
	    register uint32_t t__ = a;      \
		a=a   & 0xffff0000 | (b >> 16); \
		b=t__ & 0x0000ffff | (b << 16); \
	} while (0)

static __inline__ void
xpose4x4(uint16_t *dst, uint16_t *src)
{
ua32 *da = (ua32*)dst, *sa= (ua32*)src;
register ua32 a,b,c,d;
	a = sa[0];
	b = sa[2];
	XPOSE2x2(a,b);
	da[0] = a;
	da[2] = b;
	a = sa[1];
	b = sa[3];
	c = sa[4];
	d = sa[6];
	XPOSE2x2(a,b);
	da[4] = a;
	da[6] = b;
	XPOSE2x2(c,d);
	da[1] = c;
	da[3] = d;
	a = sa[5];
	b = sa[7];
	XPOSE2x2(a,b);
	sa[5] = a;
	sa[7] = b;
}
#endif

int
sddsTransformPage(SddsPage p, int flags)
{
int               f;
register int      j,i,jj,ii;
#ifndef VECTSZ
register uint16_t tmpu;
#endif
int16_t           *wrk = 0;

	switch ( (f = (p->flags ^ flags )) ) {
		default:
		case 0:
			/* nothing to be done */
		break;

#ifdef VECTSZ
		case FLG_LE:
		case FLG_CM | FLG_LE:
#endif
		case FLG_CM:
#ifdef VECTSZ
			if ( (f & FLG_CM) ) {
#endif
			/* transpose only     */

			if ( !wrk ) {
				wrk = malloc(p->nSamples * sizeof(*wrk) * NCOLS);
			}

			/* This algorithm is ugly and probably not most efficient,
			 * but who cares...
			 */
			if ( (flags & FLG_CM) ) {
				for ( j=jj=0; j<p->nSamples; jj+=NCOLS,j++ ) {
					for ( i=ii=0; i<NCOLS; ii+=p->nSamples,i++ ) {
						wrk[i+jj] = p->data[ii+j];
					}
				}
			} else {
				for ( j=jj=0; j<p->nSamples; jj+=NCOLS,j++ ) {
					for ( i=ii=0; i<NCOLS; ii+=p->nSamples,i++ ) {
						wrk[ii+j] = p->data[i+jj];
					}
				}
			}
			memcpy(p->data, wrk, p->nSamples * sizeof(*wrk) * NCOLS);
#ifdef VECTSZ
			}
			if ( (f & FLG_LE) ) {
				vswab(p);
			}
#endif
		break;

#ifndef VECTSZ
		case FLG_LE:
			vswab(p);
		break;

		case FLG_CM | FLG_LE:
			/* transpose & byte-swap */

			if ( !wrk ) {
				wrk = malloc(p->nSamples * sizeof(*wrk) * NCOLS);
			}

			/* This algorithm is ugly and probably not most efficient,
			 * but who cares...
			 */
			if ( (flags & FLG_CM) ) {
				for ( j=jj=0; j<p->nSamples; jj+=NCOLS,j++ ) {
					for ( i=ii=0; i<NCOLS; ii+=p->nSamples,i++ ) {
						tmpu = p->data[ii+j];
						tmpu = (tmpu>>8) | (tmpu<<8);
						wrk[i+jj] = tmpu;
					}
				}
			} else {
				for ( j=jj=0; j<p->nSamples; jj+=NCOLS,j++ ) {
					for ( i=ii=0; i<NCOLS; ii+=p->nSamples,i++ ) {
						tmpu = p->data[i+jj];
						tmpu = (tmpu>>8) | (tmpu<<8);
						wrk[ii+j] = tmpu;
					}
				}
			}
			memcpy(p->data, wrk, p->nSamples * sizeof(*wrk) * NCOLS);
		break;
#endif
	}
	if ( wrk )
		free(wrk);
	p->flags = flags;
	return 0;
}

/* transform into desired format */
int
sddsDatFormat(SddsFileDat d, int flags)
{
SddsPage p;
int      rval;
	for ( p = d->pages; p; p = p->next ) {
		if ( (rval = sddsTransformPage(p, flags)) )
			return rval;
	}
	return 0;
}

#ifdef DO_MAIN

static void
pri(SddsFileDat d)
{
SddsPage p;
int      pp;
int      i,j0,j1,j2,j3,iinc;
	for ( pp=0, p=d->pages; p; p=p->next,pp++ ) {
		printf("Page %u\n",pp);
		if ( p->flags & FLG_CM ) {
			iinc = NCOLS;
			j0   = 0; j1   = 1; j2   = 2; j3   = 3;
		} else {
			iinc = 1;
			j0   = 0; j1   = p->nSamples;
			j2   = j1 + p->nSamples;
			j3   = j2 + p->nSamples;
		}
		for ( i=0; i<p->nSamples*iinc; i+=iinc ) {
			printf(
#if 1
				"%6i %6i %6i %6i\n",
#else
				"%06x %06x %06x %06x\n",
#endif
					p->data[i+j0],
					p->data[i+j1],
					p->data[i+j2],
					p->data[i+j3]);
		}
	}
}

int
main(int argc, char **argv)
{
char *args[NCOLS] = {0};
char *fnam    = 0;
int  ch;
int  fst=0;
int  lst=-1,i;
int  ncols = 0;
int  nsamples = -1;
char *cols = 0, *colp;
SddsFileDat  d = 0;
SddsPage     p;
int  flags  = FLG_CM;
int  tflags;

	if ( !bigEndian() )
		flags |= FLG_LE;

	tflags = flags;

	while ( (ch = getopt(argc, argv, "f:1:l:c:N:F:t:")) > 0 ) {
		switch ( ch ) {
			case 'f': fnam = optarg;
			break;
			case '1': sscanf(optarg,"%i",&fst);
			break;
			case 'l': sscanf(optarg,"%i",&lst);
			break;
			case 'N': sscanf(optarg,"%i",&nsamples);
			break;
			case 'F': sscanf(optarg,"%i",&flags);
			break;
			case 't': sscanf(optarg,"%i",&tflags);
			break;
			case 'c':
				free(cols);
				ncols = 0;
				if ( (cols = strdup(optarg)) ) {
					colp = cols;
					while ( colp && *colp && ncols < sizeof(args)/sizeof(args[0]) ) {
						args[ncols++]= ',' == *colp ? 0 : colp;
						if ( (colp = strchr(colp,',')) ) {
							*colp = 0;
							colp++;
						}
					}

				}
			break;
		}
	}

	if ( !fnam ) {
		fprintf(stderr,"%s: need -f <filename> option\n", argv[0]);
		return 1;
	}

	d = sddsFileSlurp(
			fnam,
			args[0], args[1], args[2], args[3],
			fst, lst,
			nsamples,
			flags
		);

	if ( d ) {
		pri(d);
		if ( tflags != flags ) {
			printf("Transforming [%s, %s] -> [%s, %s]\n",
				 flags & FLG_CM ? "CM" : "RM",
				 flags & FLG_LE ? "LE" : "BE",
				tflags & FLG_CM ? "CM" : "RM",
				tflags & FLG_LE ? "LE" : "BE"
			);
			sddsDatFormat(d,tflags);
			pri(d);
		}
	}


	sddsDatClean(d);
	d = 0;

	free(cols);
	return 0;
}

#endif
