/* $Id: wavBuf.c,v 1.3 2010/01/18 19:30:31 strauman Exp $ */

#include <epicsInterrupt.h>
#include "wavBuf.h"

#include <string.h>
#include <stdlib.h>
#include <dbScan.h>

#define NUM_BUFHDS (WAV_BUF_NUM_SLOTS*WAV_BUF_NUM_KINDS*3)

WavBufRec wavBufPool[NUM_BUFHDS] = { {0} };
WavBuf    wavBufFreeList = 0;

WavBuf    wavBufQ[WAV_BUF_NUM_SLOTS][WAV_BUF_NUM_KINDS] = { {0} };
dbCommon *wavBufR[WAV_BUF_NUM_SLOTS][WAV_BUF_NUM_KINDS] = { {0} };

WavBufAsyncCb wavBufAsyncCbs[WAV_BUF_NUM_KINDS] = { 0 };

void
wavBufInit()
{
static int i = 0;
	if ( i )
		return;
	for (i=0; i<NUM_BUFHDS-1; i++) {
		wavBufPool[i].data = &wavBufPool[i+1];
	}
	wavBufPool[i].data = 0;
	wavBufFreeList = wavBufPool;
}

/* Allocate an empty buffer head */
WavBuf
wavBufAlloc()
{
WavBuf  rval;
int key = epicsInterruptLock();

	if ( (rval = wavBufFreeList) ) {
		wavBufFreeList = rval->data;
	}
	epicsInterruptUnlock(key);

	if ( rval ) {
		memset(rval, 0, sizeof(*rval));
	}

	return rval;
}

void
wavBufFreeHdr(WavBuf buf)
{
int key;

	if ( !buf )
		return;

	memset(buf,0,sizeof(*buf));
	key = epicsInterruptLock();
		buf->data = wavBufFreeList;
		wavBufFreeList = buf;
	epicsInterruptUnlock(key);
}

void
wavBufFree(WavBuf buf)
{
int doFreeHdr;

	if ( !buf )
		return;

	if ( ! buf->free ) {
		/* If they don't provide a free routine use 'free' */
		free(buf->data);
		free(buf->data1);
		doFreeHdr = 1;
	} else {
		doFreeHdr = (0 == buf->free(buf));
	}
	if ( doFreeHdr )
		wavBufFreeHdr(buf);
		
}

static WavBuf
wavBufSwap(unsigned slot, int kind, WavBuf b)
{
int        key;
WavBuf     oldb = 0;

	if ( slot >= WAV_BUF_NUM_SLOTS || kind < 0 || kind >= WAV_BUF_NUM_KINDS )
		return b;

	key  = epicsInterruptLock();
	oldb = wavBufQ[slot][kind];
	wavBufQ[slot][kind] = b;
	epicsInterruptUnlock(key);

	return oldb;
}


int
wavBufPost(unsigned slot, int kind, WavBuf b)
{
int    rval;
WavBuf oldb;

	if ( !b )
		return -1;

	oldb = wavBufSwap(slot, kind, b);

	/* failure -- invalid arguments */
	rval = ( oldb == b ) ? -1 : 0;

	if ( wavBufR[slot][kind] ) {
		scanOnce( wavBufR[slot][kind] );
	}

	wavBufFree(oldb);

	return rval;
}

long
wavBufRegisterRecord(dbCommon *prec, unsigned s, int k)
{
	if ( s >= WAV_BUF_NUM_SLOTS || k < 0 || k >= WAV_BUF_NUM_KINDS )
		return -1;
	wavBufR[s][k] = prec;
	return 0;
}

WavBuf
wavBufGet(unsigned slot, int kind)
{
	return wavBufSwap(slot, kind, 0);
}

int
wavBufAsyncCbRegister(unsigned kind, WavBufAsyncCb cb)
{
	if ( kind >= WAV_BUF_NUM_KINDS ) {
		epicsPrintf("wavBufAsyncCbRegister: slot # too big (> %u)\n", WAV_BUF_NUM_KINDS - 1);
		return -1;
	}
	if ( wavBufAsyncCbs[kind] && cb ) {
		epicsPrintf("wavBufAsyncCbRegister: slot #%u already used\n", kind);
		return -1;
	}
	wavBufAsyncCbs[kind] = cb;
	return 0;
}

int
wavBufAsyncStart(dbCommon *prec, void **ppvt, unsigned slot, unsigned kind, WavBuf wb)
{
	
	if ( kind >= WAV_BUF_NUM_KINDS || ! wavBufAsyncCbs[kind] || !wb )
		return -1;

	(*wavBufAsyncCbs[kind])(prec, ppvt, slot, kind, wb);
	return 0;
}

WavBuf
wavBufAsyncComplete(dbCommon *prec, void **ppvt, unsigned slot, unsigned kind)
{
	if ( kind >= WAV_BUF_NUM_KINDS || ! wavBufAsyncCbs[kind] )
		return 0;

	return (*wavBufAsyncCbs[kind])(prec, ppvt, slot, kind, 0);
}


