#include <drvPadUdpComm.h>

#ifndef TEST_ONLY
#if defined(__rtems__)
#include <drvMrfEr.h>
#else
#include <devMrfEr.h>
#endif

DrvPadUdpCommHWTime
drvPadUdpCommHWTime(void)
{
epicsUInt32 now;
	ErGetTicks(0, &now);
	return now;
}

DrvPadUdpCommHWTime
drvPadUdpCommHWBaseTime(void)
{
	return 0;
}

#else

#include <time.h>

DrvPadUdpCommHWTime
drvPadUdpCommHWTime_default(void)
{
struct timespec now;
uint64_t now_l;
	clock_gettime( CLOCK_MONOTONIC, &now );
	now_l = ((uint64_t)now.tv_sec) * 1000000000ULL + now.tv_nsec;
	return (DrvPadUdpCommHWTime)now_l;
}

DrvPadUdpCommHWTime
drvPadUdpCommHWTime(void)
__attribute__((weak, alias("drvPadUdpCommHWTime_default")));

DrvPadUdpCommHWTime
drvPadUdpCommHWBaseTime(void)
__attribute__((weak, alias("drvPadUdpCommHWTime_default")));

#endif
