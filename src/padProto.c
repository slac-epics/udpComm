#include <udpComm.h>
#include <padProto.h>

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>

#define DEBUG_PROTOHDL 1
#define DEBUG_REPLYCHK 2

#define DEBUG 0

#ifdef DEBUG
int padProtoDebug = DEBUG;
#endif

int
padStreamStart(PadRequest req, PadStrmCommand cmd, int me, uint32_t hostip)
__attribute__((weak, alias("padStreamStartNotimpl")));

int
padStreamStartNotimpl(PadRequest req, PadStrmCommand cmd, int me, uint32_t hostip)
{
	return -ENOSYS;
}

int
padStreamPet(PadRequest req, uint32_t hostip)
__attribute__((weak, alias("padStreamPetNotimpl")));

int
padStreamPetNotimpl(PadRequest req, uint32_t hostip)
{
	return -ENOSYS;
}

int
padStreamStop(uint32_t hostip)
__attribute__((weak, alias("padStreamStopNotimpl")));

int
padStreamStopNotimpl(uint32_t hostip)
{
	return -ENOSYS;
}

int
padStreamSim(PadSimCommand scmd, uint32_t hostip)
__attribute__((weak, alias("padStreamSimNotimpl")));

int
padStreamSimNotimpl(PadSimCommand scmd, uint32_t hostip)
{
	return -ENOSYS;
}


int
padProtoHandler(PadRequest req_p, int me, int *killed_p, uint32_t peerip)
{
int        		chnl = me;
int        		n;
PadCommand 		cmd;
PadReply   		rply;
int32_t			err  = 0;

	if ( PADPROTO_VERSION3 != req_p->version ) {
		return -1;
	}

	n = req_p->nCmds;
	if ( n <= 0 ) {
		/* address individual channels or broadcast */
		if ( PADREQ_BCST != n && -n != chnl )
			return 0;
		chnl = 0; /* first slot */
	} else {
		if ( chnl >= n ) {
			/* 'me' not contained in array */
			return 0;
		}
	}
#ifdef DEBUG
	if ( (padProtoDebug & DEBUG_PROTOHDL) ) {
		printf("padProtoHandler: received request for channel %i on slot #%i (", me, chnl);
			if ( PADREQ_BCST == n )
				printf("BCST");
			else
				printf("of %i", n <= 0 ? 1 : n);
		printf(")\n");
	}
#endif
	chnl *= req_p->cmdSize;

	cmd = (PadCommand)&req_p->data[chnl];

	switch ( (cmd->type & ~PADCMD_QUIET) ) {
		default:			/* unknown command */
#ifdef DEBUG
			if ( (padProtoDebug & DEBUG_PROTOHDL) ) {
			}
#endif
			fprintf(stderr,"padProtoHandler: Unknown request %i\n",cmd->type);	
			err = -EBADMSG;
			break;

		case PADCMD_NOP:	/* ignore  */
#ifdef DEBUG
			if ( (padProtoDebug & DEBUG_PROTOHDL) ) {
				printf("padProtoHandler: NOP command\n");
			}
#endif
			return 0;

		case PADCMD_ECHO:	/* echo request */
#ifdef DEBUG
			if ( (padProtoDebug & DEBUG_PROTOHDL) ) {
				printf("padProtoHandler: ECHO command\n");
			}
#endif
			break;

		case PADCMD_ECHO | PADCMD_RPLY: /* echo reply */
			/* UNIMPLEMENTED */
			fprintf(stderr,"padProtoHandler: ECHO REPLY unimplemented\n");
			err = -ENOSYS;
			break;

		case PADCMD_STRM:
#ifdef DEBUG
			if ( (padProtoDebug & DEBUG_PROTOHDL) ) {
				printf("padProtoHandler: STRM command\n");
			}
#endif
			err = padStreamStart(req_p, (PadStrmCommand)cmd, me, peerip);
			break;

		case PADCMD_SPET:
#ifdef DEBUG
			if ( (padProtoDebug & DEBUG_PROTOHDL) ) {
				printf("padProtoHandler: SPET command\n");
			}
#endif
			err = padStreamPet(req_p, peerip);
			break;

		case PADCMD_STOP:
			err = padStreamStop(peerip);
#ifdef DEBUG
			if ( (padProtoDebug & DEBUG_PROTOHDL) ) {
				printf("padProtoHandler: STOP command (err = %"PRIi32")\n", err);
			}
#endif
			break;	

		case PADCMD_SIM:
			padStreamPet(req_p, peerip);
			err = padStreamSim( (PadSimCommand) cmd, peerip);
#ifdef DEBUG
			if ( (padProtoDebug & DEBUG_PROTOHDL) ) {
				printf("padProtoHandler: SIM command\n");
			}
#endif
			break;	


		case PADCMD_KILL:
			*killed_p = 1;
			break;
	}

	if ( cmd->type & PADCMD_QUIET ) {
		/* they don't want a reply */
		return 0;
	}

	rply         = (PadReply)req_p;
	rply->type   = cmd->type | PADCMD_RPLY;
	rply->nBytes = htons(sizeof(*rply));
	rply->chnl   = me;
	if ( err < 0 ) {
		err = err < -255 ? 255 : -err;
	} else {
		err = 0;
	}
	rply->stat = err;
	/* leave other fields untouched */

	return 1;	/* packet should be sent back */
}

volatile int padudpkilled = 0;


int
padUdpHandler(uint32_t mcaddr, int port, int me, int tout_ms, int (*poll_cb)(void*), void *cb_arg)
{
int         err;
int         sd;
UdpCommPkt  p;
uint32_t	peerip;

	if ( 0 == port )
		port = PADPROTO_PORT;

	if ( (sd = udpCommSocket(port)) < 0 ) {
		return sd;
	}

	if ( mcaddr && (err = udpCommJoinMcast(sd, mcaddr)) ) {
		fprintf(stderr,"padUdpHandler() unable to join MC address 0x%08"PRIx32" (err %i)\n",ntohl(mcaddr),err);
		return err;
	}

	if ( tout_ms <= 0 )
		tout_ms = 1000; /* ~ 1s */

	while ( !padudpkilled ) {
		if ( (p = udpCommRecvFrom(sd, tout_ms, &peerip, 0)) ) {
			err = padProtoHandler((PadRequest)udpCommBufPtr(p), me, (int*)&padudpkilled, peerip);
			if ( 0 < err ) {
				/* OK to send packet */
				udpCommReturnPacket(p, ntohs(((PadReply)udpCommBufPtr(p))->nBytes));
			} else {
				/* release buffer */
				udpCommFreePacket(p);
				if ( err < 0 ) {
					fprintf(stderr,"padProtoHandler returned error %i\n",err);
				}
			}
		} else {
			if ( poll_cb && poll_cb(cb_arg) ) {
				break;
			}
		}
	}

	padudpkilled = 0;
	err          = 0;
	
	udpCommClose(sd);
	return err;	
}

#define CMDSIZE 16
#define TIMEOUT 20	/* ms */
#define RETRIES 2

int
padRequest(int sd, int who, int type, uint32_t xid, uint32_t tsHi, uint32_t tsLo, void *cmdData, UdpCommPkt *wantReply, int timeout_ms)
{
PadRequest  req;
PadReply    rep;
PadCommand	cmd;
int			rval;
UdpCommPkt  p;

	if ( who > 50 )
		return -EINVAL;

	if ( 0 == ( p = udpCommAllocPacket() ) )
		return -ENOMEM;

	req          = udpCommBufPtr( p );
	cmd          = (PadCommand)req->data;

	req->version = PADPROTO_VERSION3;
	req->nCmds   = who < 0 ? PADREQ_BCST : -who;

	req->xid     = htonl(xid);

	req->timestampHi = htonl(tsHi);
	req->timestampLo = htonl(tsLo);

	/* Add command-specific parameters */
	switch ( PADCMD_GET(cmd->type = type) ) {
		default:
			req->cmdSize = sizeof(PadCommandRec);
		break;

		case PADCMD_STRM:
			/* cmdData is a 'start command' struct */
			memcpy(cmd, cmdData, sizeof(PadStrmCommandRec));
			cmd->type    = type;
			req->cmdSize = sizeof(PadStrmCommandRec);
		break;

		case PADCMD_SIM:
			memcpy(cmd, cmdData, sizeof(PadSimCommandRec));
			cmd->type = type;
			req->cmdSize = sizeof(PadSimCommandRec);
		break;
	}

	if ( (rval = udpCommSendPkt(sd, p, sizeof(*req) + req->cmdSize)) < 0 ) {
		fprintf(stderr,"padRequest: send failed -- %s\n",strerror(-rval));
		return rval;
	}

	p = 0;

	if ( !(type & PADCMD_QUIET) ) {
		int retry = RETRIES;

		do {
		/* wait for reply */

			if ( (p = udpCommRecv(sd, timeout_ms)) ) {
				rep = (PadReply)udpCommBufPtr(p);
				if ( rep->type == (type | PADCMD_RPLY) && rep->xid == htonl(xid) ) {
					if ( wantReply )
						*wantReply = p;
					else
						udpCommFreePacket(p);
					return 0;
				} else {
#ifdef DEBUG
					if ( (padProtoDebug & DEBUG_REPLYCHK) ) {
						/* hand them the buffer for inspection */
						if (wantReply) {
							*wantReply = p;
							p = 0;
						}
					}
#endif
					udpCommFreePacket(p);
					return -EBADMSG;
				}
			}

		} while ( retry-- > 0 );

		return -ETIMEDOUT;

	}
		
	return 0;	
}
