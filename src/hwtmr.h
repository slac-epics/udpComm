#ifndef LANIPBASIC_HW_TIMER_H
#define LANIPBASIC_HW_TIMER_H

#include <stdint.h>

#if defined(__mcf5200__)
/* Note: benchmark/hw timer must be initialized! */
#include <mcf5282/mcf5282.h>

static __inline__ uint32_t 
Read_hwtimer()
{
	return MCF5282_TIMER3_DTCN;
}

#elif (defined(__powerpc__) || defined(__PPC__)) && defined(__rtems__)

#include <rtems.h> /*PPC_Get_timebase_register*/

static __inline__ uint32_t 
Read_hwtimer()
{
uint32_t tb;
	CPU_Get_timebase_low(tb);
	return tb;
}

static __inline__ uint64_t
Read_hwtimer_long()
{
	return PPC_Get_timebase_register();
}

#elif defined(__rtems__) && defined(__i386__)

#include <libcpu/cpuModel.h>

static __inline__ uint32_t
Read_hwtimer()
{
	return rdtsc();
}

static __inline__ uint64_t
Read_hwtimer_long()
{
	return rdtsc();
}

#else

static __inline__ uint32_t 
Read_hwtimer()
{
	return 0xdeadbeef;
}

static __inline__ uint64_t 
Read_hwtimer_long()
{
	return 0xdeadbeefaffecafeLL;
}

#endif


#endif
