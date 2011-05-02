#include <epicsExit.h>
#include <epicsThread.h>
#include <registryFunction.h>
#include <iocsh.h>

#include <drvPadUdpComm.h>

static const struct iocshArg chArg[] = {
	{ "channel",  iocshArgInt },
	{ "nsamples", iocshArgInt }
};

static const struct iocshArg *chArgp[] = {
	&chArg[0],
	&chArg[1]
};

#define DECL1(nm,argp) \
static iocshFuncDef nm##Desc = { \
	#nm,   \
	1, \
	argp   \
}; \
static void nm##Func(const iocshArgBuf *args) \
{ \
	nm(args[0].ival); \
}

#define DECL2(nm,argp) \
static iocshFuncDef nm##Desc = { \
	#nm,   \
	2, \
	argp   \
}; \
static void nm##Func(const iocshArgBuf *args) \
{ \
	nm(args[0].ival, args[1].ival); \
}


#define REG(nm) iocshRegister( &nm##Desc, nm##Func )

DECL1(drvPadUdpCommStrmStartReq,     chArgp)
DECL1(drvPadUdpCommStrmStopReq,      chArgp)
DECL2(drvPadUdpCommStrmSetNSamples,  chArgp)
DECL2(drvPadUdpCommStrmSetChannels, chArgp)

int main(int argc, char *argv[])
{
	REG(drvPadUdpCommStrmStartReq);
	REG(drvPadUdpCommStrmStopReq);
	REG(drvPadUdpCommStrmSetNSamples);
	REG(drvPadUdpCommStrmSetChannels);
	if ( argc >= 2 ) {
		iocsh( argv[1] );
		epicsThreadSleep( 0.2 );
	}
	iocsh( 0 );
	epicsExit( 0 );
	return 0;
}
