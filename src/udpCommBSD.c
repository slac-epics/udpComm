/* $Id: udpCommBSD.c,v 1.13 2009/12/05 19:28:15 strauman Exp $ */

#ifndef BSDSOCKET
#define BSDSOCKET
#endif

/* Glue layer to send padProto over ordinary UDP sockets */

#include <udpComm.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#if 0 /* avoid dependency :-( */
/* to get alignment only */
#include <lanIpProto.h>
#else
#define LAN_IP_BASIC_PACKET_ALIGNMENT 32
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>


#define DO_ALIGN(x,a) (((uintptr_t)(x) + ((a)-1)) & ~((a)-1))
#define BUFALIGN(x)   DO_ALIGN(x,LAN_IP_BASIC_PACKET_ALIGNMENT)

/* maintain same alignment of data-area 
 * which is seen when using lanIpBasic.
 * Pad # of bytes consumed by ethernet, IP
 * and UDP headers.
 */
#define PADSZ         (16+20+8)

typedef struct {
	char   data[UDPCOMM_PKTSZ];
	struct sockaddr sender;
	int             rx_sd;
	void            *raw_mem;
} __attribute__((may_alias)) UdpCommBSDPkt;

UdpCommPkt
udpCommAllocPacket()
{
void          *p_raw;
UdpCommBSDPkt *p;

	p_raw      = malloc(sizeof(*p) + PADSZ + LAN_IP_BASIC_PACKET_ALIGNMENT-1);

	if ( !p_raw )
		return 0;

	p          = (UdpCommBSDPkt*)(BUFALIGN(p_raw) + PADSZ);
	p->raw_mem = p_raw;
	p->rx_sd   = -1;
	return p;
}

int
udpCommSocket(int port)
{
int                sd = socket(AF_INET, SOCK_DGRAM, 0);
struct sockaddr_in me;

	if ( sd < 0 )
		return sd;

	memset(&me, 0, sizeof(me));
	me.sin_family = AF_INET;
	me.sin_port   = htons(port);

	if ( bind(sd, (void*)&me, sizeof(me)) ) {
		close(sd);
		return -errno;
	}

	return sd;
}

UdpCommPkt
udpCommRecvFrom(int sd, int timeout_ms, uint32_t *ppeerip, uint16_t *ppeerport)
{
struct timeval     tv;
fd_set             fds;
UdpCommBSDPkt      *p;
struct sockaddr_in __attribute__((may_alias)) *sa;
socklen_t          len;

	tv.tv_sec  = timeout_ms/1000;
	tv.tv_usec = 1000*(timeout_ms % 1000);
	FD_ZERO(&fds);
	FD_SET(sd, &fds);

	if ( select(sd+1, &fds, 0, 0, &tv) <= 0 )
		return 0;

	if ( !(p = udpCommAllocPacket()) )
		return 0;

	sa         = (struct sockaddr_in __attribute__((may_alias)) *)&p->sender;
	len        = sizeof( p->sender );

	if ( recvfrom(sd, p->data, sizeof(p->data), 0, &p->sender, &len) < 0 ) {
		udpCommFreePacket(p);
		return 0;
	} else {
		if ( ppeerip )
			*ppeerip = sa->sin_addr.s_addr;
		if ( ppeerport )
			*ppeerport = ntohs(sa->sin_addr.s_addr);
	}
	p->rx_sd = sd;
	return p;
}

void
udpCommFreePacket(UdpCommPkt p)
{
	if ( p )
		free(((UdpCommBSDPkt*)p)->raw_mem);
}

UdpCommPkt
udpCommRecv(int sd, int timeout_ms)
{
	return udpCommRecvFrom(sd, timeout_ms, 0, 0);
}

void
udpCommReturnPacket(UdpCommPkt p0, int len)
{
UdpCommBSDPkt *p = (UdpCommBSDPkt *)p0;
	sendto( p->rx_sd, p->data, len, 0, &p->sender, sizeof(p->sender));
}

int
udpCommConnect(int sd, uint32_t diaddr, int port)
{
struct sockaddr_in they;
	
	memset(&they, 0, sizeof(they));
	they.sin_family        = AF_INET;
	they.sin_addr.s_addr   = diaddr;
	they.sin_port          = htons(port);
	return connect(sd, (struct sockaddr*)&they, sizeof(they));
}

int
udpCommSend(int sd, void *buf, int len)
{
	return send(sd, buf, len, 0);
}

int
udpCommSendPkt(int sd, UdpCommPkt pkt, int len)
{
UdpCommBSDPkt *p = (UdpCommBSDPkt*) pkt;
int           rval;

	rval = send(sd, p->data, len, 0);
	udpCommFreePacket(p);
	return rval;
}

int
udpCommSendPktTo(int sd, UdpCommPkt pkt, int len, uint32_t dipaddr, int port)
{
UdpCommBSDPkt *p = (UdpCommBSDPkt*) pkt;
int           rval;

union {
	struct sockaddr_in sin;
	struct sockaddr    sa;
} dst;

	dst.sin.sin_family      = AF_INET;
	dst.sin.sin_addr.s_addr = dipaddr;
	dst.sin.sin_port        = htons(port);
	rval = sendto(sd, p->data, len, 0, &dst.sa, sizeof(dst.sin));
	udpCommFreePacket(p);
	return rval;
}


/* can tweak this in special cases so select incoming IF... */
static uint32_t udpCommMcastIfAddr = INADDR_ANY;

void
udpCommSetIfMcastInp(uint32_t ifipaddr)
{
	udpCommMcastIfAddr = ifipaddr;
}

static int mc_doit(int sd, uint32_t mcaddr, int cmd)
{
#ifdef __linux__
struct ip_mreqn ipm;
#else /* rtems, BSD (?) */
struct ip_mreq  ipm;
#endif

	ipm.imr_multiaddr.s_addr = mcaddr;
#ifdef __linux__
	ipm.imr_address.s_addr   = udpCommMcastIfAddr;
	ipm.imr_ifindex          = 0;
#else
	ipm.imr_interface.s_addr = udpCommMcastIfAddr;
#endif

	if ( setsockopt(sd, IPPROTO_IP, cmd, &ipm, sizeof(ipm)) ) {
		return -errno;
	}

	return 0;
}

int
udpCommJoinMcast(int sd, uint32_t mcaddr)
{
	return mc_doit(sd, mcaddr, IP_ADD_MEMBERSHIP);
}

int
udpCommLeaveMcast(int sd, uint32_t mcaddr)
{
	return mc_doit(sd, mcaddr, IP_DROP_MEMBERSHIP);
}

int
udpCommSetIfMcastOut(int sd, uint32_t ifipaddr)
{
#ifdef __linux__
struct ip_mreqn arg;

	memset(&arg, 0, sizeof(arg));
#define mcifa arg.imr_address
#else
struct in_addr  arg;
#define mcifa arg
#endif

	mcifa.s_addr = ifipaddr;

	if ( setsockopt(sd, IPPROTO_IP, IP_MULTICAST_IF, &mcifa, sizeof(mcifa)) ) {
		return -errno;
	}

	return 0;
}

int
udpCommClose(int sd)
{
	return close(sd);
}

void *
udpCommBufPtr(UdpCommPkt p)
{
	return p;
}
