/* $Id: fidProcess.h,v 1.6 2010/01/19 01:20:34 strauman Exp $ */
#ifndef GENERIC_FID_PROCESS_H
#define GENERIC_FID_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

extern int fidProcessHasBeam;

extern unsigned fidHeartbeat;

/* find out if there should be beam during this cycle */
static __inline__ int
fidHasBeam()
{
	return fidProcessHasBeam;
}

extern volatile DrvPadUdpCommHWTime fidTimeBaseline;

/*
 * Install a callback to obtain a time-baseline for the fiducial;
 * This callback runs in the context of the evrTask.
 * The callback also determines whether (at least part of the 
 * machine) carries beam for 'this' 120Hz pulse).
 */
void
fidTimeInstall_generic(void);

void
fidTestProcess(void);

#ifdef __cplusplus
};
#endif
#endif
