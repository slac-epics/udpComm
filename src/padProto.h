/* $Id: padProto.h,v 1.9 2011/04/27 22:18:32 strauman Exp $ */

#ifndef PADPROTO_DEF_H
#define PADPROTO_DEF_H

/* Communication Protocol with PAD over dedicated network */

#include <stdint.h>
#include <udpComm.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Hijack (hopefully unused) port numbers...
 *
 * hde-lcesrvr-1   14936/tcp  hde-lcesrvr-1
 * hde-lcesrvr-1   14936/udp  hde-lcesrvr-1
 * hde-lcesrvr-2   14937/tcp  hde-lcesrvr-2
 * hde-lcesrvr-2   14937/udp  hde-lcesrvr-2
 *
 * However, any port may be used as long
 * as both partners agree about it.
 */
#define PADPROTO_PORT		14936
#define PADPROTO_STRM_PORT	14937

#define PADCMD_RPLY  ((int8_t)(1<<7)) 	/* reply ORs request with msb      */
#define PADCMD_QUIET ((int8_t)(1<<6))	/* requestor doesn't want a reply  */

#define	PADCMD_NOP  ((int8_t) 0)
#define	PADCMD_ECHO ((int8_t) 1)
#define	PADCMD_STRM ((int8_t) 2)
#define	PADCMD_SPET ((int8_t) 3)		/* 'pet' just update timestamps    */
#define	PADCMD_STOP ((int8_t) 4)
#define PADCMD_KILL ((int8_t)15)
#define PADCMD_SIM  ((int8_t) 5)        /* generate simulated response     */

/* mask off the flag bits */
#define PADCMD_GET(type)  ((type) & ~(PADCMD_RPLY | PADCMD_QUIET))

typedef struct PadCommandRec_ {
	int8_t		type;			/* PADCMD_XX                           */
	int8_t		sdata[3];		/* commands with small data needs      */
	uint32_t	ldata[];		/* word sized commands                 */
} __attribute__((may_alias)) PadCommandRec, *PadCommand;

#define PADCMD_STRM_FLAG_LE	1	/* They want little-endian data    */
#define PADCMD_STRM_FLAG_CM	2	/* They want column-major  data    */
#define PADCMD_STRM_FLAG_32 4   /* Data are 32-bit                 */
#define PADCMD_STRM_FLAG_C1 8   /* Data are one channel only       */

#define PADCMD_SIM_FLAG_NOSEND 1 /* Don't send simulated waveform; just save requested amplitudes */

#define PADRPLY_STRM_FLAG_TYPE_SET(x)	(((x)&7)<<4)
#define PADRPLY_STRM_FLAG_TYPE_GET(fl)	(((fl)>>4)&7)
#define PADRPLY_STRM_NUM_KINDS          8

/* Sample size is sizeof(int16_t) */
#define PADRPLY_STRM_NCHANNELS	4
#define PADRPLY_STRM_LD_SZ(r)   (((r)->strm_cmd_flags & PADCMD_STRM_FLAG_32) ? 2 : 1)

/* Number of samples in a packet */
#define PADRPLY_STRM_NSAMPLES(r)                                           \
	((ntohs((r)->nBytes) - sizeof(PadReplyRec)) >> PADRPLY_STRM_LD_SZ(r))

/* Number of channels transmitted in the stream. Note that a fragment
 * of a row-major stream only contains data for a single channel. The
 * number returned by PADRPLY_STRM_NCHANNELS() conveys the number of
 * the channels in the stream, not in a particular fragment.
 */
#define PADRPLY_STRM_CHANNELS_IN_STRM(r) (((r)->strm_cmd_flags & PADCMD_STRM_FLAG_C1) ? 1 : PADRPLY_STRM_NCHANNELS)

/* How many channels are in a single fragment?
 * In CM format, this is equal to the number of channels in the stream.
 * However, in RM format there is only a single channel in a fragmented
 * stream.
 */
#define PADRPLY_STRM_CHANNELS_IN_FRAG(r) \
	( (   ! ((r)->strm_cmd_flags & PADCMD_STRM_FLAG_CM) &&	PADRPLY_STRM_IS_FRAGMENTED(r) ) \
     ?  1 : PADRPLY_STRM_CHANNELS_IN_STRM(r) )

#define PADRPLY_STRM_IS_FRAGMENTED(r) (!! (r)->strm_cmd_idx)

typedef struct PadStrmCommandRec_ {
	int8_t		type;			/* PADCMD_XX                           */
	uint8_t		flags;			/* echoed in 'spec[0]' of reply        */
	uint16_t	port;			/* port where to send data             */
	uint32_t	nsamples;		/* # samples per channel               */
} __attribute__((may_alias)) PadStrmCommandRec, *PadStrmCommand;

typedef struct PadSimCommandRec_ {
	int8_t		type;
	int8_t		flags;
	int8_t		sdata[2];
	int32_t		a,b,c,d;
} __attribute__((may_alias)) PadSimCommandRec, *PadSimCommand;

/* Not used anymore
#define PADPROTO_VERSION1		0x31
#define PADPROTO_VERSION2		0x32
 */
#define PADPROTO_VERSION3		0x33

#define PADREQ_BCST	(-128) /* Address all channels with a single command */

/* Note: cmdSize must be a multiple of 4 (dword-aligned)               */
typedef struct PadRequestRec_ {
	uint8_t		version;		/* Protocol version                    */
	int8_t		nCmds;			/* number of channels,-128: broadcast  */
	int8_t		cmdSize;		/* size of command structures          */
	int8_t		r1;				/* reserved                            */
	uint32_t	timestampHi;
	uint32_t	timestampLo;
	uint32_t	xid;            /* transaction 'id'                    */
	uint32_t	r2;				/* align to 16-bytes (incl. IP hdrs)   */
	uint8_t		data[];
} __attribute__((may_alias)) PadRequestRec, *PadRequest;
/* appended here are |nCmds| PadCommandRecs */

/* We have a hard time to squeeze additional information into a stream
 * reply which is required for fragmented streams.
 * A fragmented, four-channel, row-major stream should convey some
 * information about its layout -- otherwise, the receiver has to
 * buffer all packets (or implicitly know about the layout) before
 * it can figure it out.
 *
 * A fragmented stream is transmitted as follows:
 *
 *   strm_cmd_idx:   Contains the packet index and a 'more-fragments' (MF)
 *                   flag. 'MF' is set on all but the last fragment. A
 *                   fragmented stream can be detected simply by testing
 *                   strm_cmd_idx which can be zero only for a non-fragmented
 *                   stream. HOWEVER: ALWAYS use the macros defined below
 *                   since the implementation may change!
 *
 * The implementation restricts fragmented streams to a payload size
 * of 1024 bytes. Hence, the number of channels and samples must be
 * chosen to meet that restriction.
 *
 * Column-major layout: CM can be transmitted in a straightforward
 *                   manner.
 *
 * Row-major layout: This format requires some attention. In order to keep
 *                   things simple the implementation requires that
 *                   row-boundaries *must* coincide with packet boundaries.
 *                   Together with the 1k payload requirement this means
 *                   that sample numbers per channel must be multiples of
 *                   512 (16bit data) or 256 (32-bit data), respectively.
 *
 *                   The number of channels the stream contains is indicated
 *                   by the '_C1' flag (but the user should ALWAYS use the
 *                   macros to figure out the number of channels rather than
 *                   testing the flag directly since the implementation may
 *                   change).
 *
 *                   HOWEVER: A packet of a fragmented, row-major stream 
 *                   only contains data associated with a single channel.
 *                   In order to find out how many packets make up a
 *                   row of data the user must either know this or know
 *                   the total number of samples (which is known once
 *                   all fragments arrive and can be divided by the number
 *                   of channels in the stream).
 */

typedef struct PadReplyRec_ {
	uint8_t		version;		/* Protocol version                    */
	int8_t		type;			/* reply to command of 'type'          */
	int8_t		chnl;			/* channel sending the reply           */
	uint8_t		stat;           /* status code (errno)                 */
	uint32_t	timestampHi;
	uint32_t	timestampLo;
	uint32_t	xid;            /* transaction 'id'                    */
	uint16_t	nBytes;			/* size of reply                       */
	uint8_t		spec[2];		/* 2 bytes of command specific data    */
	uint8_t		data[];         /* aligned on 16-byte boundary         */
} __attribute__((may_alias)) PadReplyRec, *PadReply;

#define strm_cmd_flags	spec[0]
#define strm_cmd_idx    spec[1]
/* 'More fragments' flag */
#define PADRPLY_STRM_CMD_IDX_MF  0x80
#define PADRPLY_STRM_CMD_IDX_GET(idx) ((idx) & ~PADRPLY_STRM_CMD_IDX_MF)

/* Handle Protocol Request
 *    'req_p': The request. This may be modified by this routine to form a reply.
 *       'me': Channel number to look for ('our' channel/slot # in the packet).
 * 'killed_p': Pointer to a variable that is set if a 'KILL' command was received.
 *   'peerip': IP address of the requestor (network byte order).
 *
 * RETURNS: < 0 on error (-errno), 0 or 1 on success. If 1 is returned then the
 *          request was transformed into a reply and should be returned to the requestor.
 */
int
padProtoHandler(PadRequest req_p, int me, int *killed_p, uint32_t peerip);

/* Handle padProto requests on 'port' for channel/slot 'chnl' until an error
 * occurs or a KILL command is received (the padUdpHandler() must be idle
 * for 'tout_ms' before it returns due to a KILL command).
 *
 * If the port number is zero, then the predefined port PADPROTO_PORT
 * (padProto.h) will be used.
 *
 * The 'mcaddr' argument may be set to an IPV4 multicast address
 * (in NETWORK-byte order) which will then be joined.
 *
 * If there is no padProto activity for 'tout_ms' then the the 'poll_cb'
 * (if non-null) is executed and its return value examined. 'padUdpHandler'
 * returns if 'poll_cb()' returns nonzero.
 *
 * If 'tout_ms' is less than or equal to zero then a default timeout
 * of 1000ms is used.
 *
 * RETURNS: 0 on success -errno on error
 */
int
padUdpHandler(uint32_t mcaddr, int port, int me, int tout_ms, int (*poll_cb)(void*), void *cb_arg);

/* Send a request to 'connected' peer
 *        'sd': socket descriptor
 *      'chnl': target channel/slot (may be broadcast)
 *      'type': command
 *       'xid': transaction ID - value is returned in reply
 * 'tsHi/tsLo': timestamp
 *   'cmdData': parameter to command
 * 'wantReply': if non-NULL then q reply from the peer is requested and
 *              returned in *wantReply.
 *'timeout_ms': how many ms to wait for a reply.
 *
 * RETURNS: 0 on success, -errno on error.
 *
 * NOTES: This routine does not provide the full functionality
 *        the protocol allows. It is not possible to send different
 *        commands to different channels with this routine.
 */
int
padRequest(int sd, int chnl, int type, uint32_t xid, uint32_t tsHi, uint32_t tsLo, void *cmdData, UdpCommPkt *wantReply, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
