#ifndef SDDS_READER_H
#define SDDS_READER_H

#include <stdint.h>

#define NCOLS   4

#define FLG_CM (1<<0)
#define FLG_LE (1<<1)
#define FLG_32 (1<<2)

typedef union SddsDataP_ {
	int16_t *s;
	int32_t *l;
	void    *r;
} SddsDataP;

typedef struct SddsPageRec_ {
	struct SddsPageRec_ *next;
	int                 nSamples;
	int                 flags;
	SddsDataP           data; /* pointer to aligned data area */
	char                buf[];
} SddsPageRec, *SddsPage;

typedef struct SddsFileDatRec_ {
	int      numPages;
	SddsPage pages;
} SddsFileDataRec, *SddsFileDat;

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
	);

void
sddsDatClean(SddsFileDat wrk);

/* transform into desired format (transposing is not efficient!) */

/* single page */
int
sddsTransformPage(SddsPage p, int flags);

/* all pages */
int
sddsDatFormat(SddsFileDat d, int flags);

#endif
