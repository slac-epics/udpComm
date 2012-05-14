
#include <epicsTime.h>
#include <errlog.h>

#ifndef TEST_ONLY
#include <evrTime.h>
#include <evrPattern.h>
#endif

#include <drvPadUdpComm.h>

volatile DrvPadUdpCommHWTime fidTimeBaseline   = 0;
volatile int                 fidProcessHasBeam = 0;
volatile unsigned            fidHeartbeat      = 0;

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

unsigned
fidProcessGeneric(epicsTimeStamp *time_s_p, unsigned time_slot_mask)
{
epicsTimeStamp             time_s;
#ifndef TEST_ONLY
/* This is actually an array type */
evrModifier_ta             modifier_a;
unsigned long              patternStatus;
int                        st;
#endif

	if ( !time_s_p )
		time_s_p = &time_s;

	fidProcessHasBeam = 1;

#ifndef TEST_ONLY
	/* only interested in 120Hz... */
	st = evrTimeGetFromPipeline( time_s_p, evrTimeCurrent, modifier_a, &patternStatus, 0, 0, 0);
	if ( st ) {
		epicsPrintf("drvPadUdpComm: (fidTimeGetBaseline_generic()) epicsTimeGetFromPipeline failed (status %i)\n", st);
		
		return 0;
	}

	if ( (time_slot_mask & modifier_a[E_MOD2]) ) {
		/* This is it! */
		fidTimeBaseline = drvPadUdpCommHWTime();

		fidHeartbeat++;

		drvPadUdpCommSendStartAll( time_s_p );
	}

	return (modifier_a[E_MOD2] & TIMESLOT_MASK);
#else
	epicsTimeGetCurrent( time_s_p );
	drvPadUdpCommSendStartAll( time_s_p );
	return 1;
#endif
}

void
fidTimeGetBaseline_generic(void *unused)
{
	fidProcessGeneric(0, TIMESLOT1_MASK | TIMESLOT4_MASK);
}

void
fidTimeInstall_generic(void)
{
	fidProcessHasBeam = 1;
#ifndef TEST_ONLY
	evrTimeRegister(fidTimeGetBaseline_generic, 0);
#endif
}
