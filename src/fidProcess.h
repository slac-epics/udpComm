/* $Id: fidProcess.h,v 1.2 2012/05/14 15:14:09 strauman Exp $ */
#ifndef GENERIC_FID_PROCESS_H
#define GENERIC_FID_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

extern volatile int fidProcessHasBeam;

extern volatile unsigned fidHeartbeat;

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
 *
 * The generic callback executes fidProcessGeneric for timeslots 1 & 4.
 */
void
fidTimeInstall_generic(void);

void
fidTestProcess(void);

/*
 * Obtain a time-stamp and pattern info for this fiducial.
 * If the bit corresponding to the current time-slot is set
 * in 'time_slot_mask' then 
 *  - fidHeartBeat is incremented
 *  - fidTimeBaseline is set to the current DrvPadUdpCommHWTime.  
 *  - drvPadUdpCommSendStartAll() is executed, passing the time-stamp.
 *
 * time_s_p: pointer to an epicsTimeStamp; may be NULL if the timestamp
 *           is not of interest to the caller.
 *
 * active_timeslots: bitmask selecting time-slots considered 'active',
 *           i.e., when the above actions are to be performed.
 *
 * RETURNS: current timeslot bit (MOD2 & TIMESLOT_MASK) or zero if 
 *          no valid pattern could be obtained.
 */

unsigned
fidProcessGeneric(epicsTimeStamp *time_s_p, unsigned active_timeslots);

#ifdef __cplusplus
};
#endif
#endif
