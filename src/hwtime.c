#include <drvPadUdpComm.h>

#include <time.h>

DrvPadUdpCommHWTime
drvPadUdpCommHWTime_default(void)
{
struct timespec now;
DrvPadUdpCommHWTime       now_ns;
const DrvPadUdpCommHWTime ns = (DrvPadUdpCommHWTime)1000000000ULL;

	clock_gettime( CLOCK_MONOTONIC, &now );
	now_ns = ((DrvPadUdpCommHWTime)now.tv_sec) * ns + (DrvPadUdpCommHWTime)now.tv_nsec;
	return now_ns;
}

DrvPadUdpCommHWTime
drvPadUdpCommHWTime(void)
__attribute__((weak, alias("drvPadUdpCommHWTime_default")));

DrvPadUdpCommHWTime
drvPadUdpCommHWBaseTime(void)
__attribute__((weak, alias("drvPadUdpCommHWTime_default")));
