
#include <epicsTime.h>

#ifndef TEST_ONLY
#include <evrTime.h>
#include <evrPattern.h>
#endif

#include <drvPadUdpComm.h>

volatile DrvPadUdpCommHWTime fidTimeBaseline   = 0;
int                          fidProcessHasBeam = 0;
unsigned                     fidHeartbeat      = 0;

void
fidTestProcess(void) __attribute__((weak, alias("fidTestProcess_default")));

void
fidTestProcess_default()
{
epicsTimeStamp ts;
	epicsTimeGetCurrent(&ts);
	drvPadUdpCommSendStartAll(&ts);
}

#define E_MOD2 1
#define E_MOD5 4

void
fidTimeGetBaseline_generic(void *unused)
{
#ifndef TEST_ONLY
/* This is actually an array type */
evrModifier_ta             modifier_a;
epicsTimeStamp             time_s;
unsigned long              patternStatus;
int                        st;
DrvPadUdpCommHWTime        now, diff;

	fidProcessHasBeam = 1;

	/* only interested in 120Hz... */
	st = evrTimeGetFromPipeline( &time_s, evrTimeCurrent, modifier_a, &patternStatus, 0, 0, 0);
	if ( st ) {
		epicsPrintf("drvPadUdpComm: (fidTimeGetBaseline_generic()) epicsTimeGetFromPipeline failed (status %i)\n", st);
		
		return;
	}

	if ( (TIMESLOT1_MASK | TIMESLOT4_MASK) & modifier_a[E_MOD2] ) {
		/* This is it! */
		now = drvPadUdpCommHWTime();
		diff = now - fidTimeBaseline;
		fidTimeBaseline = now;

		fidHeartbeat++;

		drvPadUdpCommSendStartAll(&time_s);
	}
#endif
}

void
fidTimeInstall_generic(void)
{
	fidProcessHasBeam = 1;
#ifndef TEST_ONLY
	evrTimeRegister(fidTimeGetBaseline_generic, 0);
#endif
}
