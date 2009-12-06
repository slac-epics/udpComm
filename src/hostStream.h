#ifndef HOST_STREAM_H
#define HOST_STREAM_H

#ifdef USE_SDDS
int
padStreamSddsSetup(const char *fspec, int pFst, int pLst);
#endif

int
padStream(int32_t a, int32_t b, int32_t c, int32_t d);

int
padStreamSimulated();

#endif
