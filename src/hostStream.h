#ifndef HOST_STREAM_H
#define HOST_STREAM_H

#include <sys/time.h>

#ifdef USE_SDDS
int
padStreamSddsSetup(const char *fspec, int pFst, int pLst);
#endif

int
padStream(int32_t a, int32_t b, int32_t c, int32_t d);

int
padStreamSimulated(void *unused);

extern time_t padStreamTimeoutSecs;

#endif
