/* $Id: udpComm.h,v 1.5 2010/04/16 22:22:37 strauman Exp $ */
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

/* Create */
int
udpCommSocket(int port);

/* Close  */
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
 */
int
udpCommConnect(int sd, uint32_t diaddr, int port);

/* Receive a packet */
UdpCommPkt
udpCommRecv(int sd, int timeout_ms);

/* Receive a packet and sender information
 *
 * NOTE: port is in host, IP address in network byte order.
 */
UdpCommPkt
udpCommRecvFrom(int sd, int timeout_ms, uint32_t *ppeerip, uint16_t *ppeerport);

/* Allocate a packet (for sending with udpCommSendPktTo) */
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
 */
int
udpCommSendPkt(int sd, UdpCommPkt pkt, int len);

/*
 * As above but send to specified peer
 * (unconnected socket only).
 *
 * NOTE: 'dipaddr' is the peer's IP address in *network* byte order
 *       'port'    is the peer's port number in *host*   byte order
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
 * RETURNS: zero on success, -errno on failure.
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
 * RETURNS: zero on success, nonzero (-errno) on
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
