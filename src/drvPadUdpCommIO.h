#ifndef DRV_PAD_UDPCOMM_IO_API_H
#define DRV_PAD_UDPCOMM_IO_API_H
/* $Id: drvPadUdpCommIO.h,v 1.1 2010/01/19 02:16:13 strauman Exp $ */

/* An ugly driver defining I/O 'methods' for drvPadUdpComm.
 * 
 * The reason for the existence of this API is purely historic.
 *
 * First, there were only PAD digitizers communicating with the
 * BPM IOC over a dedicated UDP/IP/Ethernet stack and thus
 * 'udpComm' was born. 'drvPadUdpComm' was the lowest-layer
 * driver dealing with PAD communication.
 * Higher level drivers (for specific types of BPMs, e.g., stripline
 * vs. cavity BPMs) would register a callback table with
 * drvPadUdpComm() who would dispatch calls to the appropriate
 * high-level driver e.g., when doing signal processing of the raw
 * data received from the PAD.
 *
 * Then, the VME digitizer appeared. In an attempt to use as much
 * existing code as possible a udpComm emulation layer was created
 * so that essentially nothing within the VME BpmApp needed to be
 * changed.
 *
 * However, at some point it was decided that 'udpComm' would also
 * be employed for fast-feedback and possibly other communication
 * purposes.
 * Since a IOC with a VME digitizer would now also need 'true' udpComm
 * the emulation could no longer be present since it would conflict
 * with the 'true' udpComm module.
 * 
 * Thus the advent of this new interface providing indirection so
 * that drvPadUdpComm() can either talk to a VME digitizer or use
 * PADs over true udpComm.
 *
 * A much more elegant solution which would already be half-ways
 * implemented would be introducing a true, file-descriptor based
 * driver interface at the bottom of drvPadUdpComm.
 * drvPadUdpComm could then just read/write from a file descriptor
 * which -- under the hood - could map to either the VME digitizer
 * driver or drvUdpSock (file-descriptor based driver to udpComm).
 *
 * However, the ordinary read/write interface would always require
 * copying all of the read data into a user buffer.
 * Largely because we want to avoid that overhead we go down
 * a different path and introduce this callback table, sorry.
 *
 * T. Straumann, 12/2009
 */

#include <stdint.h>

#include <udpComm.h>

typedef struct DrvPadUdpCommIORec_ {
	int                 (*open)     (int port);
	int                 (*close)    (int fd);
	int                 (*connect)  (int fd, uint32_t peer_ip, int peer_port);
	int                 (*send)     (int fd, void    *buf_p,   int len);
	UdpCommPkt          (*recv)     (int fd, int timeout_ms);
	void *              (*bufptr)   (UdpCommPkt);
	UdpCommPkt          (*alloc)    ();
	void                (*free)     (UdpCommPkt);
	/* This last one is ugly but does different things
	 * with the VME digitizer as opposed to the PAD.
	 * Again: this is historic evolution not clean design from scratch...
	 */
	int                 (*padIoReq) (int fd, int chnl, int type,
	                                 uint32_t xid, uint32_t tsHi, uint32_t tsLo,
	                                 void  *cmdData, UdpCommPkt *rply,
	                                 int timeout_ms);
} DrvPadUdpCommIORec, *DrvPadUdpCommIO;

extern DrvPadUdpCommIO drvPadVmeCommIO __attribute__((weak));

#endif
