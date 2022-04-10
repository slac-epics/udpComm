/* $Id: udpCommBSD.c,v 1.12 2016/03/22 04:36:40 bhill Exp $ */

/* Glue layer to send padProto over ordinary UDP sockets */

#include <udpComm.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <limits.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <pthread.h>

#define ERRNONEG (errno ? -errno : -1)

static pthread_mutex_t  udpcomm_Mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t  freelst_Mtx = PTHREAD_MUTEX_INITIALIZER;

#define __LOCK()   do { if ( pthread_mutex_lock( & udpcomm_Mtx ) ) abort(); }  while ( 0 )
#define __UNLOCK() pthread_mutex_unlock( & udpcomm_Mtx )

#define __LOCKLST()   do { if ( pthread_mutex_lock( & freelst_Mtx ) ) abort(); }  while ( 0 )
#define __UNLOCKLST() pthread_mutex_unlock( & freelst_Mtx )

#define DO_ALIGN(x,a) (((uintptr_t)(x) + ((a)-1)) & ~((a)-1))
#define BUFALIGN(x)   DO_ALIGN(x,UDPCOMM_DATA_ALGN)

#define ISMCAST(a) (((a) & 0xf0000000) == 0xe0000000)

typedef struct {
	struct sockaddr_in sina;
} __attribute__((may_alias)) sin_a;

static int ISBCAST(uint32_t addr)
{
struct ifaddrs *ifa, *p;
sin_a          *psa;
int             rval = 1;

	if ( getifaddrs(&ifa) ) {
		return -1;
	}
	
	for ( p=ifa; p; p=p->ifa_next ) {
		if (    (IFF_BROADCAST & p->ifa_flags)
             &&  p->ifa_broadaddr
             &&  AF_INET == p->ifa_broadaddr->sa_family ) {
			psa = (sin_a*)p->ifa_broadaddr;
			if ( psa->sina.sin_addr.s_addr == addr ) {
				rval = 0;
				break;
			}
		}
	}

	freeifaddrs(ifa);

	return rval;
}

/* maintain same alignment of data-area 
 * which is seen when using lanIpBasic.
 * Pad # of bytes consumed by ethernet, IP
 * and UDP headers.
 */
#define PADSZ         (UDPCOMM_DATA_ALGN - UDPCOMM_DATA_ALGN_OFF)

typedef struct {
	char   data[UDPCOMM_PKTSZ];
	struct sockaddr sender;
	int             rx_sd;
	void            *raw_mem;
	int             refcnt;
	int             data_sz;
} __attribute__((may_alias)) UdpCommBSDPkt;

typedef union PktBuf_ {
	union PktBuf_   *next;
	char             raw[sizeof(UdpCommBSDPkt) + PADSZ + UDPCOMM_DATA_ALGN - 1];
} PktBuf;

#ifndef OPEN_MAX
/* Some systems do not define this. Use
 * a reasonable value - the udpSockCreate()
 * routine checks against overflow.
 */
/* linuxRT systems are getting more than 35 file descriptors
 * easily when accessing the ATCA crates. We had to increase
 * OPEN_MAX to allow bigger sd.
 */
#ifdef __linux__
#define OPEN_MAX 50 
#else
#define OPEN_MAX 20
#endif
#endif

static struct {
	uint32_t dipaddr;
	uint16_t dport;
} sdaux[OPEN_MAX] = { {0} };

static PktBuf *freeList = 0;

UdpCommPkt
udpCommAllocPacket()
{
PktBuf        *p_raw;
UdpCommBSDPkt *p;

	__LOCKLST();
		if ( (p_raw = freeList) ) {
			freeList = p_raw->next;
		}
	__UNLOCKLST();

	if ( ! p_raw ) {
		p_raw = malloc(sizeof(*p_raw));
	}

	if ( !p_raw )
		return 0;

	p          = (UdpCommBSDPkt*)(BUFALIGN(p_raw->raw) + PADSZ);
	p->raw_mem = p_raw;
	p->rx_sd   = -1;
	p->refcnt  =  1;
	p->data_sz = 0;
	return p;
}

int
udpCommSocket(int port)
{
int                sd = socket(AF_INET, SOCK_DGRAM, 0);
struct sockaddr_in me;
int                err, yes;

        if ( sd < 0 )
		return ERRNONEG;

	if ( sd >= OPEN_MAX ) {
		fprintf(stderr,"INTERNAL ERROR. OPEN_MAX is defined as %d in udpCommBSD.c. The socket file descriptor sd was opened as %d. sd can't be > OPEN_MAX.\n", OPEN_MAX, sd);
		close(sd);
		return -ENFILE;
	}

	/* using sdaux should be thread-safe (udpCommClose doesn't
	 * access sdaux). The OS gives us a new unique SD so that we
	 * automatically 'own' its slot in sdaux.
	 */

	sdaux[sd].dipaddr = 0;

	memset(&me, 0, sizeof(me));
	me.sin_family = AF_INET;
	me.sin_port   = htons(port);

	yes = 1;
	if ( setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) ) {
		err = ERRNONEG;
		close(sd);
		return err;
	}

	yes = 1;
	if ( setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) ) {
		err = ERRNONEG;
		close(sd);
		return err;
	}


#if 1
	yes = 1;
	if ( setsockopt(sd, IPPROTO_IP, IP_MULTICAST_LOOP, &yes, sizeof(yes)) ) {
		err = ERRNONEG;
		close(sd);
		return err;
	}
#endif


	if ( bind(sd, (void*)&me, sizeof(me)) ) {
		err = ERRNONEG;
		close(sd);
		return err;
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
int                got;

	tv.tv_sec  = timeout_ms/1000;
	tv.tv_usec = 1000*(timeout_ms % 1000);
	FD_ZERO(&fds);
	FD_SET(sd, &fds);

	/* timeout_ms < 0 indicates blocking forever */
	if ( select(sd+1, &fds, 0, 0, timeout_ms >= 0 ? &tv : 0) <= 0 )
		return 0;

	if ( !(p = udpCommAllocPacket()) )
		return 0;

	sa         = (struct sockaddr_in __attribute__((may_alias)) *)&p->sender;
	len        = sizeof( p->sender );

	if ( (got = recvfrom(sd, p->data, sizeof(p->data), 0, &p->sender, &len)) < 0 ) {
		udpCommFreePacket(p);
		return 0;
	} else {
		if ( ppeerip )
			*ppeerip = sa->sin_addr.s_addr;
		if ( ppeerport )
			*ppeerport = ntohs(sa->sin_port);
	}
	p->rx_sd   = sd;
	p->data_sz = got;
	return p;
}

void
udpCommFreePacket(UdpCommPkt p)
{
int c;
UdpCommBSDPkt *pkt = (UdpCommBSDPkt*)p;
PktBuf        *raw;

	if ( pkt ) {
		__LOCK();
		c = --(pkt->refcnt);
		__UNLOCK();
		if ( 0 == c ) {
			__LOCKLST();
				raw       = (PktBuf *) pkt->raw_mem;
				raw->next = freeList;
				freeList  = raw;
			__UNLOCKLST();
		} else if ( c < 0 ) {
			abort();
		}
	}
}

void
udpCommRefPacket(UdpCommPkt p)
{
UdpCommBSDPkt *pkt = (UdpCommBSDPkt*)p;
	__LOCK();
	pkt->refcnt++;
	__UNLOCK();
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
int                rval;

	memset(&they, 0, sizeof(they));

	__LOCK();
	/* Do not connect to a multicast/broadcast destination
	 * but remember destination in sdaux.
	 */
	if ( ISMCAST(ntohl(diaddr)) || 0 == ISBCAST(diaddr) ) {
		sdaux[sd].dipaddr = diaddr;
		sdaux[sd].dport   = htons(port);
#ifdef __linux__
		/* linux docs say AF_UNSPEC is used to disconnect; 
		 * RTEMS/BSD seems to use AF_INET/INADDR_ANY
		 */
		they.sin_family   = AF_UNSPEC;
#else
		they.sin_family   = AF_INET;
#endif
	} else {
		sdaux[sd].dipaddr      = 0;
		they.sin_family        = AF_INET;
		they.sin_addr.s_addr   = diaddr ? diaddr : INADDR_ANY;
		they.sin_port          = htons(port);

	}
	rval = connect(sd, (struct sockaddr*)&they, sizeof(they)) ? ERRNONEG : 0;

	__UNLOCK();

	return rval;
}

int
udpCommSend(int sd, void *buf, int len)
{
int rval;
	__LOCK();
	if ( sdaux[sd].dipaddr ) {
		union {
			struct sockaddr_in ia;
			struct sockaddr    sa;
		} dst;
		dst.ia.sin_family      = AF_INET;
		dst.ia.sin_addr.s_addr = sdaux[sd].dipaddr;
		dst.ia.sin_port        = sdaux[sd].dport;
		rval = sendto(sd, buf, len, 0, &dst.sa, sizeof(dst.ia));
	} else {
		rval = send(sd, buf, len, 0);
	}
	__UNLOCK();
	return rval < 0 ? (ERRNONEG) : rval ;
}

int
udpCommSendPkt(int sd, UdpCommPkt pkt, int len)
{
UdpCommBSDPkt *p = (UdpCommBSDPkt*) pkt;
int           rval;

	rval = udpCommSend( sd, p->data, len );

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
	return rval < 0 ? (ERRNONEG) : rval;
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
		return ERRNONEG;
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
		return ERRNONEG;
	}

	return 0;
}

int
udpCommSetMcastTTL(int sd, uint32_t ttl )
{
	if ( setsockopt(sd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) ) {
		return ERRNONEG;
	}
	return 0;
}
  

int
udpCommClose(int sd)
{
	return close(sd) ? ERRNONEG : 0;
}

void *
udpCommBufPtr(UdpCommPkt p)
{
	return p;
}

int
udpCommBufSize(UdpCommPkt p)
{
	return ((UdpCommBSDPkt *)p)->data_sz;
}

