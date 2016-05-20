/* $Id: udpComm.c,v 1.7 2016/03/22 04:36:40 bhill Exp $ */

#include <lanIpBasic.h>
#include <netinet/in.h>

/* The reason for having two very similar APIs (udpSock and udpComm) are
 * mostly historical. 
 * However, udpComm is a slightly higher abstraction which is also implemented
 * over BSD sockets.
 * OTOH, udpSock has stronger typing and offers more features.
 */

#include <udpComm.h>

#if UDPCOMM_ALIGNMENT > LAN_IP_BASIC_PACKET_ALIGNMENT
#error "lanIpBasic alignment unsufficient -- must change and rebuild lanIpBasic"
#endif

int     
udpCommSocket(int port)
{
	return udpSockCreate(port);
}

int     
udpCommClose(int sd)
{
	return udpSockDestroy(sd);
}

int     
udpCommConnect(int sd, uint32_t diaddr, int port)
{
	return udpSockConnect(sd, diaddr, port, UDPSOCK_MCPASS);
}

UdpCommPkt
udpCommAllocPacket()
{
	return udpSockGetBuf();
}

void
udpCommFreePacket(UdpCommPkt p)
{
	udpSockFreeBuf(p);
}

void
udpCommRefPacket(UdpCommPkt p)
{
	udpSockRefBuf(p);
}


int
udpCommJoinMcast(int sd, uint32_t mc_addr)
{
	return udpSockJoinMcast(sd, mc_addr);
}

int
udpCommLeaveMcast(int sd, uint32_t mc_addr)
{
	return udpSockLeaveMcast(sd, mc_addr);
}

int
udpCommSetIfMcastOut(int sd, uint32_t mc_addr)
{
	return udpSockSetIfMcast(sd, mc_addr);
}

int
udpCommSetMcastTTL(int sd, uint32_t ttl )
{
	/* setsockOpt IP_MULTICAST_TTL is not supported and
	 * apparently not needed for RTEMS */
	return 0;
}

void
udpCommSetIfMcastInp(uint32_t mc_addr)
{
	udpSockMcastIfAddr = mc_addr;
}

static inline int ms2ticks(int ms)
{
    if ( ms > 0 ) {
        rtems_interval rate;
        rtems_clock_get(RTEMS_CLOCK_GET_TICKS_PER_SECOND, &rate);
        if ( ms > 50000 ) {
            ms /= 1000;
            ms *= rate;
        } else {
            ms *= rate;
            ms /= 1000;
        }
        if ( 0 == ms ) {
            ms = 1;
        }
    }
    return ms;
}

UdpCommPkt
udpCommRecv(int sd, int timeout_ms)
{
	return udpSockRecv(sd, ms2ticks(timeout_ms));
}

UdpCommPkt
udpCommRecvFrom(int sd, int timeout_ms, uint32_t *ppeerip, uint16_t *ppeerport)
{
LanIpPacket rval;

	rval = udpSockRecv(sd, timeout_ms);

	if ( rval ) {
		if ( ppeerip )
			*ppeerip   = lpkt_ip(rval).src;
		if ( ppeerport )
			*ppeerport = ntohs(lpkt_udp(rval).sport);
	}
	return rval;
}

void *
udpCommBufPtr(UdpCommPkt p)
{
	return udpSockUdpBufPayload((LanIpPacket)p);
}

/* HACK -- this should really go into lanIpBasic */
int
udpCommBufSize(UdpCommPkt p)
{
    return ntohs(lpkt_udp_hdrs((LanIpPacket)p).udp.len) - sizeof(UdpHeaderRec);
}


int
udpCommSend(int sd, void *payload, int payload_len)
{
	return _udpSockSendTo_internal(sd, 0, payload, payload_len, 0, 0);
}


int
udpCommSendPkt(int sd, UdpCommPkt b, int payload_len)
{
	return _udpSockSendTo_internal(sd, b, 0, payload_len, 0, 0);
}

int
udpCommSendPktTo(int sd, UdpCommPkt b, int payload_len, uint32_t ipaddr, int dport)
{
	return _udpSockSendTo_internal(sd, b, 0, payload_len, ipaddr, dport);
}

void
udpCommReturnPacket(UdpCommPkt p, int len)
{
IpBscIf pif;
	if ( (pif = udpSockGetBufIf(p)) ) {

		/* Set length first (udpSockHdrsReflect calculates IP hdr checksum) */
        len += sizeof(UdpHeaderRec);
        lpkt_udp((LanIpPacket)p).len = htons(len);
        len += sizeof(IpHeaderRec);
        lpkt_ip((LanIpPacket)p).len  = htons(len);

		/* Point headers back to sender and recalculate IP hdr checksum     */
        udpSockHdrsReflect(&lpkt_udp_hdrs((LanIpPacket)p));

		/* Send off                                                         */
		lanIpBscSendBufRawIp(pif, p);
	}
}
