#ifndef  WAVBUF_H
#define  WAVBUF_H
/* $Id: wavBuf.h,v 1.2 2008-11-25 01:39:43 strauman Exp $ */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <epicsTime.h>
#include <dbCommon.h>

#define WAV_BUF_NUM_SLOTS	32
#define WAV_BUF_NUM_KINDS	5

/* Types */
#define WavBufNone  	0
#define	WavBufInt16		((1<<4) | sizeof(int16_t))
#define WavBufInt32		((2<<4) | sizeof(int32_t))
#define	WavBufFloat     ((3<<4) | sizeof(float))
#define	WavBufDouble    ((4<<4) | sizeof(double))
/* split-complex; display amplitude */
#define	WavBufCplx      ((5<<4) | sizeof(float))
/* split-complex; display phase     */

#define WavBufFlagCM    (1<<0)

#define wavBufTypeSize(x) ((x)&0xf)

typedef struct WavBufRec_ {
	void           *data;
	void           *data1;
	uint8_t        type;
	uint8_t        flags;
	uint16_t       stride;
	uint16_t 	   m,n; /* dimension of waveform array */
	void           *usrData;
	/* Release data and header; if free returns zero
	 * then wavBufFreeHdr is called after 'free' returns
	 * and the buffer header is returned to the
	 * general-purpose free list.
	 */
	int           (*free)(struct WavBufRec_ *pb);
	epicsTimeStamp ts;
} WavBufRec, *WavBuf;

/* may return 0 if no data is
 * available (=> READ_ALARM)
 *
 * If a non-null item is returned,
 * the user must free it with
 * wavBufFree();
 */
WavBuf
wavBufGet(unsigned slot, int kind);

/* Allocate an empty buffer head */
WavBuf
wavBufAlloc();

/* Release a buffer header; this only
 * provided for special purposes.
 * Normally, providing a 'free' method
 * and letting it return 0 results
 * in wavBufFreeHdr() being called
 * from wavBufFree().
 */
void
wavBufFreeHdr(WavBuf buf);

/*
 * Release buffer header and attached
 * data via the buffer header's 'free'
 * member.
 */
void
wavBufFree(WavBuf buf);

int
wavBufPost(unsigned slot, int kind, WavBuf b);

void
wavBufInit();

/*
 * Register a record; wavBufPost() issues a "scanOnce()"
 * request for the registered record.
 *
 * RETURNS: 0 on success, nonzero if s/k are out of range.
 */
long
wavBufRegisterRecord(dbCommon *prec, unsigned s, int k);

/*
 * registered callback is called twice: 
 *  1st time by wavBufAsyncStart()
 *  2nd time by wavBufAsyncComplete() with NULL 'wb' arg.
 *
 * RETURN: processed (or new) waveform; i.e., the callback
 *         facility may take an 'original' wavBuf, transform
 *         it into a new one, free the old one and return the
 *         transformed one to wavBufAsyncComplete().
 */
typedef WavBuf (*WavBufAsyncCb)(dbCommon *prec, void **ppvt,unsigned slot,unsigned kind, WavBuf wb);

int
wavBufAsyncCbRegister(unsigned kind, WavBufAsyncCb cb);

int
wavBufAsyncStart(dbCommon *prec, void **ppvt, unsigned slot, unsigned kind, WavBuf wb);

WavBuf
wavBufAsyncComplete(dbCommon *prec, void **pvt, unsigned slot, unsigned kind);

#ifdef __cplusplus
};
#endif

#endif
