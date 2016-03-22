/* $Id: udpComm.h,v 1.7 2014/07/31 19:52:46 strauman Exp $ */
#ifndef UDPCOMM_LAYER_H
#define UDPCOMM_LAYER_H

/* Glue for simple UDP communication over
 * either BSD sockets or simple 'udpSock'ets and
 * the lanIpBasic 'stack'.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Offset UDPCOMM_DATA_ALGN_OFF into payload
 * is guaranteed to be aligned to UDPCOMM_DATA_ALGN 
 * --- WATCH OUT: payload itself is NOT aligned.
 */
#define UDPCOMM_DATA_ALGN_OFF 4
#define UDPCOMM_DATA_ALGN     16

#ifndef UdpCommPkt
typedef void * UdpCommPkt;
#endif

/* Most routines defined in this interface return a negative
 * error status ('NEGERR') on error whereas a zero or positive
 * return value signals success.
 * The (absolute value of a) negative error code is a standard
 * 'errno' code and may be converted into a string by e.g.,
 *
 * #include <errno.h>
 * #include <string.h>
 *
 *    if ( status < 0 ) fprintf(stderr,"Message: %s\n", strerror(-status));
 */

/* Create; RETURNS: NEGERR (see above) or 0 on success */
int
udpCommSocket(int port);

/* Close; RETURNS: NEGERR (see above) or 0 on success  */
int
udpCommClose(int sd);

/* Connect socket to a peer
 *
 * NOTES: 'dipaddr' is the peer's IP address in *network* byte order
 *        'port'    is the peer's port number in *host*   byte order
 *
 *        If 'dipaddr' is a IP multicast or broadcast address then
 *        BSD 'connect' semantics are not obeyed. Instead, udpComm stores
 *        the destination address/port in memory and uses 'sendto' from
 *        an unconnected socket. (Otherwise the socket would be unable
 *        to receive since only packets from the connected peer [a
 *        MC/BC address in this case] would be accepted).
 * RETURNS: NEGERR (see above) on failure, 0 on success;
 */
int
udpCommConnect(int sd, uint32_t diaddr, int port);

/* Receive a packet; RETURNS: packet handle on success, NULL on failure */
UdpCommPkt
udpCommRecv(int sd, int timeout_ms);

/* Receive a packet and sender information
 *
 * NOTE: port is in host, IP address in network byte order.
 *
 * RETURNS: packet handle on success, NULL on failure.
 */
UdpCommPkt
udpCommRecvFrom(int sd, int timeout_ms, uint32_t *ppeerip, uint16_t *ppeerport);

/* Allocate a packet (for sending with udpCommSendPktTo)
 *
 * RETURNS: packet handle on success, NULL on failure.
 */
UdpCommPkt
udpCommAllocPacket();

/* Release packet (obtained from Recv) when done       */
void
udpCommFreePacket(UdpCommPkt p);

/* Create an additional 'reference' to a packet (increment
 * an internal reference counter).
 * udpCommFreePacket() decrements the reference count and only
 * really releases the packet buffer once the count drops to
 * zero.
 * This facility can be used if you need to re-use the same
 * packet from different software modules.
 * NOTE: Any modifications to the packet data are 'seen' by
 *       ALL users who hold a reference!
 */
void
udpCommRefPacket(UdpCommPkt p);

/* Payload size = eth MTU - ip and udp header sizes    */
#define UDPCOMM_PKTSZ (1500 - 20 - 8)

/* Obtain pointer to data area in buffer (UDP payload) */
void *
udpCommBufPtr(UdpCommPkt p);

/* Send packet to connected peer; 
 * The data in 'buf' has to be copied
 * into the 'lanIpBasic' stack (no-op
 * when using BSD sockets).
 *
 * RETURNS: NEGERR (see above) on failure, number of octets sent on success.
 */
int
udpCommSend(int sd, void *buf, int len);

/* Send packet w/o extra copy step.
 * Packet must be pre-allocated using
 * udpCommAllocPacket() and filled with
 * data (into the user area).
 *
 * NOTE: Ownership of the packet is
 *       transferred to the stack by
 *       this call (regardless of the
 *       return value).
 *
 * RETURNS: NEGERR (see above) on failure, number of octets sent on success.
 */
int
udpCommSendPkt(int sd, UdpCommPkt pkt, int len);

/*
 * As above but send to specified peer
 * (unconnected socket only).
 *
 * NOTE: 'dipaddr' is the peer's IP address in *network* byte order
 *       'port'    is the peer's port number in *host*   byte order
 *
 * RETURNS: NEGERR (see above) on failure, number of octets sent on success.
 */
int
udpCommSendPktTo(int sd, UdpCommPkt pkt, int len, uint32_t dipaddr, int port);

/* Return packet to sender (similar to 'send'; 
 * this interface exists for efficiency reasons
 * [coldfire/lan9118]).
 */
void
udpCommReturnPacket(UdpCommPkt p, int len);

/* Join and leave a MC group. This actually affects the interface
 * not just the socket 'sd'.
 * NOTE: calls do not nest; you cannot call this twice on the
 *       same socket.
 *
 * RETURN: zero on success, NEGERR (see above) on failure.
 */

int
udpCommJoinMcast(int sd, uint32_t mc_addr);

int
udpCommLeaveMcast(int sd, uint32_t mc_addr);

/*
 * Set the outgoing interface for sending multicast
 * traffic from 'sd'. The interface with IP address
 * 'ifaddr' is used for sending all MC traffic from
 * socket 'sd'. 
 *
 * Note that the same or similar functionality 
 * can be achieved by setting up the system routing
 * tables which is more transparent, flexible and
 * IMHO preferable.
 * However, in some cases -- especially for testing --
 * setting this on a 'per-socket' basis with this
 * call is useful; particularly because (on linux
 * and other general-purpose, protected OSes) no
 * special privileges are required.
 *
 * RETURNS: zero on success, NEGERR (see above) on
 *          error.
 *
 * NOTES:   use a 'ifaddr' == 0 to remove the
 *          association of a outgoing mcast IF
 *          with a socket.
 *
 *          The 'ifipaddr' is as usual given in
 *          network-byte order.
 */
int
udpCommSetIfMcastOut(int sd, uint32_t ifipaddr);

/*
 * Set the multicast TTL
 * If TTL is left at it's default of 1, the multicast packets
 * will not leave the local subnet.
 *
 * RETURNS: zero on success, NEGERR (see above) on
 *          error.
 */
int
udpCommSetMcastTTL(int sd, uint32_t ttl );

/* Set the IP address of the receiving interface
 * (if host has multiple NICs) in network byte order.
 * Defaults to INADDR_ANY, i.e., system picks a suitable IF.
 */
void
udpCommSetIfMcastInp(uint32_t ifipaddr);

#ifdef __cplusplus
}
#endif

#endif
