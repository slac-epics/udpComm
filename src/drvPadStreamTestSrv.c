/* $Id$ */
#include <drvSup.h>
#include <epicsExport.h>
#include <errlog.h>
#include <epicsThread.h>

#include <string.h>

#include <padProto.h>
#include <padStream.h>
#include <lanIpBasicSetup.h>

unsigned drvPadStreamTestSrvChannel = 2;

unsigned drvPadStreamTestSrvKinds   = 2;
double   drvPadStreamTestSrvPeriod  = 1.0;

static void
padProtoSrv(void *arg)
{
int err;
	if ( (err = padUdpHandler( 0, 0, drvPadStreamTestSrvChannel, 0, 0, 0 )) ) {
		epicsPrintf("ERROR: padUdpHandler() aborted: %s\n", strerror(-err));
	}
}

static void
padStrmSrv(void *arg)
{
int kind;
	while ( 1 ) {
		epicsThreadSleep( drvPadStreamTestSrvPeriod );
		for ( kind = drvPadStreamTestSrvKinds - 1; kind>=0; kind-- ) {
			padStreamTest( kind );
		}
	}
}

static long report(void)
{
	epicsPrintf("Simple driver for testing PAD streaming\n");
	return 0;
}

static long init(void)
{
int err;
	if ( (err = padStreamInitialize( lanIpIf, 0, 0 )) ) {
		epicsPrintf("Fatal error: Unable to initialize stream: %s\n", strerror(-err));
		return -1;
	}
	epicsThreadMustCreate(
		"padProtoSrv",
		epicsThreadPriorityScanHigh, 
		epicsThreadGetStackSize( epicsThreadStackMedium ),
		padProtoSrv,
		0);
	epicsThreadMustCreate(
		"padStrmSrv",
		epicsThreadPriorityMax,
		epicsThreadGetStackSize( epicsThreadStackMedium ),
		padStrmSrv,
		0);
	return 0;
}


static drvet drvPadStreamTestSrv = {
	number: 2,
	report: report,
	init:   init
};

epicsExportAddress( drvet, drvPadStreamTestSrv );
