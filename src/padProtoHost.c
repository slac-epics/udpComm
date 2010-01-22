/* $Id: padProtoHost.c,v 1.3 2010/01/19 00:29:14 strauman Exp $ */


/* Wrapper program to send padProto requests */

#include <padProto.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <netdb.h>

#include <arpa/inet.h>

#include "hostStream.h"

static int verbose    = 0;

static int theChannel = 0;

void
pdump(UdpCommPkt p)
{
int i,sz;
	if (p) {
		printf("Packet Dump:");
		sz = ntohs(((PadReply)p)->nBytes);
		for (i=0; i<sz; i++) {
			if ( i%16 == 0 )
				printf("\n");
			printf("%02X ",((uint8_t*)p)[i]);
		}
		printf("\n");
	}
}

void
dumpReply(PadReply p)
{
int16_t tmp;
	printf("Got Reply (proto version 0x%02x):\n", p->version);
	printf("     Type: 0x%02x\n",           (uint8_t)p->type);
	printf("  Channel: %i\n",               p->chnl);
	tmp = ntohs(p->nBytes);
	printf("   nBytes: %"PRIi16" [payload %"PRIi16"]\n",  tmp, tmp-(int16_t)sizeof(*p));
	printf("      XID: 0x%08x\n",           ntohl(p->xid)); 
	tmp = ntohs(p->status);
	printf("   status: %i (%s)\n",          tmp, strerror(-tmp));
	printf("     spec: 0x%02x, 0x%02x\n",   p->spec[0], p->spec[1]);
}

static int
isbe()
{
union {
	uint8_t	 xx[2];
	uint16_t tst;
} endian = { xx : {0xbe, 00} };
	return (0xbe00 == endian.tst);
}

void
usage(char *nm)
{
	fprintf(stderr,"Usage: %s [-bhveLc] [-d debug_facility] [-C channel] [-l port] [-n nsamples] [-s srvr_port] [-t <stream_timeout_secs>] "
#ifdef USE_SDDS
	"[-S sdds_file:col,col,col,col] [-p start:end] "
#endif
    "[-P server_xmit_period] [-d dbg_flgs] ip:port <msg_type_int>\n",nm);
	fprintf(stderr,"          -b bcast to all channels\n");
	fprintf(stderr,"          -h print this help\n");
	fprintf(stderr,"          -v be verbose\n");
	fprintf(stderr,"          -d <facility> enable padProto debug messages (ORed bitset)\n");
	fprintf(stderr,"          -e request wrong endianness (for testing; STRM command only)\n");
	fprintf(stderr,"          -c request col-major data   (for testing; STRM command only)\n");
	fprintf(stderr,"          -L request long (32-bit) data   (STRM command only)\n");
	fprintf(stderr,"          -n <nsamples> request <nsamples> (STRM command only)\n");
	fprintf(stderr,"          -C <channel>  send to PAD # <channel>\n");
	fprintf(stderr,"          -l <port>     operate in STRM listener mode on <port>\n");
	fprintf(stderr,"          -n <port>     operate in STRM listener mode on <port>\n");
	fprintf(stderr,"          <msg_type_int> (forced to STRM if any of -[necL] options present:\n");
	fprintf(stderr,"                      0 = NOP\n");
	fprintf(stderr,"                      1 = ECHO\n");
	fprintf(stderr,"                      2 = STRM (start stream)\n");
	fprintf(stderr,"                      3 = SPET (pet stream)\n");
	fprintf(stderr,"                      4 = STOP (stop stream)\n");
	fprintf(stderr,"                      5 = SIM  (simulate BPM)\n");
	fprintf(stderr,"                     15 = KILL (terminate padUdpHandler on target)\n");
	fprintf(stderr,"          -s srvr_port  run as a padProto server\n");
#ifdef USE_SDDS
	fprintf(stderr,"          -S sdds_file:col,col,col,col SDDS file to 'play'\n");
	fprintf(stderr,"                                       specified columns.\n");
#endif
	fprintf(stderr,"          -P period_ms  transmit simulated or playback\n");
	fprintf(stderr,"                        waveforms every 'period_ms'\n");
#ifdef USE_SDDS
	fprintf(stderr,"          -p start:end  SDDS page range (1-based) to 'play'\n");
#endif
	fprintf(stderr,"          -m mcaddr     Multicast address to use in server mode\n");
}

static void
client(int sd, char *ip, int port, void *cmdp)
{
int               err;
UdpCommPkt        p = 0;
PadCommand        cmd = cmdp;

	if ( (err = udpCommConnect(sd, inet_addr(ip), port)) ) {
		fprintf(stderr,"udpCommConnect: %s",strerror(-err));
		exit(1);
	}

	if ( (err=padRequest(sd, theChannel, cmd->type, 0xdead, 0, 0, cmdp, &p, 1000)) ) {
		fprintf(stderr,">>> Sending request failed: %s <<<\n",strerror(-err));
	}

	if ( p )
		dumpReply((PadReply)p);

	udpCommFreePacket(p);
}

static void
streamdump(int sd)
{
UdpCommPkt        p;
PadReply          rply;
int16_t           err;
int               i;
int               d32;
int               sz;
union {
	int16_t *s;
	int32_t *l;
	void    *r;
}                 buf;

	while ( (p = udpCommRecv(sd, 1000000000)) ) {
		rply = (PadReply)p;
		if ( verbose ) 
			dumpReply(rply);
		if ( rply->type != (PADCMD_STRM | PADCMD_RPLY) ) {
			fprintf(stderr,"Listener: received invalid reply 0x%02x\n", (uint8_t)rply->type);
			break;
		}
		if ( (err=ntohs(rply->status)) ) {
			fprintf(stderr,"Listener: received bad reply (%i -> %s)\n", err, strerror(-err));
			break;
		}
		/* Check matching endianness */
		if ( isbe() != !(rply->strm_cmd_flags & PADCMD_STRM_FLAG_LE) ) {
			fprintf(stderr,"Listener: endianness mismatch\n");
			break;
		}


		d32 = (rply->strm_cmd_flags & PADCMD_STRM_FLAG_32) ? 1 : 0;

		sz = PADRPLY_STRM_NSAMPLES(rply);

		/* Dump packet */
		buf.r = rply->data;

		printf("// IDX: %i, TYPE %i\n",
			rply->strm_cmd_idx,
			PADRPLY_STRM_FLAG_TYPE_GET(rply->strm_cmd_flags));
		/* always write out in fortran (scilab) format with
		 * the samples going down the columns
		 */
		if ( rply->strm_cmd_flags & PADCMD_STRM_FLAG_CM ) {
			for (i=0; i<sz*4; i+=4) {
				if ( d32 ) {
					printf("%5i %5i %5i %5i\n",
							buf.l[i+0], buf.l[i+1], buf.l[i+2], buf.l[i+3]);
				} else {
					printf("%5i %5i %5i %5i\n",
							buf.s[i+0], buf.s[i+1], buf.s[i+2], buf.s[i+3]);
				}
			}
		} else {
			for (i=0; i<sz; i++) {
				if ( d32 ) {
					printf("%5i %5i %5i %5i\n",
							buf.l[i+0*sz], buf.l[i+1*sz], buf.l[i+2*sz], buf.l[i+3*sz]);
				} else {
					printf("%5i %5i %5i %5i\n",
							buf.s[i+0*sz], buf.s[i+1*sz], buf.s[i+2*sz], buf.s[i+3*sz]);
				}
			}
		}
		printf("\n\n");
		fflush(stdout);
		udpCommFreePacket(p);
	}

	if ( p ) {
		dumpReply((PadReply)p);
		udpCommFreePacket(p);
	}
}

int
main(int argc, char **argv)
{
int               sd;
int               type = -1;
int               ch;
int               i;
union {
PadStrmCommandRec stmc;
PadSimCommandRec  simc;
PadCommandRec     rawc;
}                 the_cmd;
int               port      = 0;
int               listener  = 0;
char               *col     = 0;
int               nsamples  = 8;
int               badEndian = 0;
int               d32       = 0;
int               colMajor  = 0;
int               srvrMode  = 0;
unsigned          dbg       = 0;
int               srvrPerMs = 0;
#ifdef USE_SDDS
const char        *sddsnam  = 0;
int               pgFst     = 0;
int               pgLst     = -1;
#endif
uint32_t          mcaddr    = 0;
const char *      mcgrp     = 0;
int               err;

	while ( (ch = getopt(argc, argv, "bvcehLl:n:C:s:d:S:t:P:p:m:")) > 0 ) {
		switch (ch) {
			default:
				fprintf(stderr,"Unknown option '%c'\n",ch);
				usage(argv[0]);
				exit(1);

			case 'h':
				usage(argv[0]);
				exit(0);

			case 'd':
				if ( 1!=sscanf(optarg,"%i",&dbg) ) {
					fprintf(stderr,"invalid 'debug' value: '%s'\n", optarg);
					usage(argv[0]);
					exit(1);
				}
			break;

			case 'l':
			case 's':
				if ( 1!= sscanf(optarg,"%i",&port) || port < 0 || port > 65535 ) {
					fprintf(stderr,"invalid port number: '%s'\n", optarg);
					usage(argv[0]);
					exit(1);
				}
				if ( 'l' == ch )
					listener = 1;	
				else
					srvrMode = 1;
				break;

			case 'S':
#ifdef USE_SDDS
				sddsnam = optarg;
#else
				fprintf(stderr,"program was built w/o SDDS support; recompile with -DUSE_SDDS\n");
#endif
			break;

			case 't':
				if ( 1 != sscanf(optarg, "%i", &i) ) {
					fprintf(stderr,"need <timeout> (int) argument\n");
					exit(1);
				}
				padStreamTimeoutSecs = i;
			break;

			case 'p':
#ifdef USE_SDDS
				if ( 2 != sscanf(optarg,"%i:%i",&pgFst,&pgLst) ) {
					fprintf(stderr,"need <start-page>:<end-page> range\n");
					exit(1);
				}
				if ( --pgFst < 0 ) 
					pgFst = 0;
				/* last page < 0 means -> end */
				pgLst--;

#else
				fprintf(stderr,"program was built w/o SDDS support; recompile with -DUSE_SDDS\n");
#endif
			break;

			case 'b': theChannel = -128; break;

			case 'v': verbose   = 1; break;
			case 'c': colMajor  = 1; type = PADCMD_STRM; break;
			case 'e': badEndian = 1; type = PADCMD_STRM; break;
			case 'L': d32       = 1; type = PADCMD_STRM; break;

			case 'C':
				if ( 1 != sscanf(optarg,"%i",&theChannel) || theChannel > 255 || (theChannel < 0 && -128 != theChannel) ) {
					fprintf(stderr,"invalid channel #: '%s'\n",optarg);
					usage(argv[0]);
					exit(1);
				}
			break;

			case 'P':
				if ( 1 != sscanf(optarg,"%i",&srvrPerMs) ) {
					fprintf(stderr,"invalid period (number expected): '%s'\n",optarg);
					usage(argv[0]);
					exit(1);
				}
			break;

			case 'n':
				if ( 1 != sscanf(optarg,"%i",&nsamples) ) {
					fprintf(stderr,"invalid number of samples: '%s'\n",optarg);
					usage(argv[0]);
					exit(1);
				}
				type = PADCMD_STRM;
			break;

			case 'm':
				mcgrp = optarg;
			break;
		}
	}

#ifdef USE_SDDS
	if ( sddsnam &&  padStreamSddsSetup(sddsnam,pgFst,pgLst) ) {
		fprintf(stderr,"invalid SDDS filespec\n");
		exit(1);
	}
#endif

	if ( !listener && !srvrMode ) {
		if ( argc - optind < 2 || !(col=strchr(argv[optind],':')) || (*col++=0, INADDR_NONE == inet_addr(argv[optind])) || 1 != sscanf(col,"%i",&port) || (-1 == type && 1 != sscanf(argv[optind+1],"%i",&type)) ) {
			usage(argv[0]);
			exit(1);
		}
	}

	
	if ( 0 == port )
		port = listener ? PADPROTO_STRM_PORT : PADPROTO_PORT;

	if ( srvrMode ) {
		extern int padProtoDebug;
		struct addrinfo *res = 0, *p;

		if ( mcgrp ) {
			if ( (err = getaddrinfo(mcgrp, 0, 0, &res)) ) {
				fprintf(stderr,"Unable to lookup '%s': %s\n", mcgrp, gai_strerror(err));
				if ( res )
					freeaddrinfo(res);
				exit(1);	
			} else {
				for ( p = res; p; p = p->ai_next ) {
					if ( AF_INET == p->ai_family ) {
						mcaddr = ((struct sockaddr_in*)p->ai_addr)->sin_addr.s_addr;
						break;
					}
				}
				freeaddrinfo(res);
				if ( ! mcaddr ) {
					fprintf(stderr,"Unable to find '%s': ???\n", mcgrp);
					exit(1);
				}
			}
		}

		padProtoDebug = dbg;
		padUdpHandler(mcaddr, port, theChannel, srvrPerMs, padStreamSimulated ,0);
		/* should never return here */
		perror("padUdpHandler failed");
		exit(1);
	}


	sd = udpCommSocket(listener ? port : 0);

	if ( sd < 0 ) {
		fprintf(stderr,"udpCommSocket: %s",strerror(-sd));
		exit(1);
	}

	if ( listener ) {
		streamdump(sd);
	} else {
		switch ( (the_cmd.rawc.type = type) ) {
			case PADCMD_STRM:
				the_cmd.stmc.flags = (!isbe() ^ (badEndian == 1)) ? PADCMD_STRM_FLAG_LE : 0;
				if ( d32 )
					the_cmd.stmc.flags |= PADCMD_STRM_FLAG_32;
				the_cmd.stmc.port     = htons(port+1); /* should be PADPROTO_STRM_PORT */
				the_cmd.stmc.nsamples = htonl(nsamples);

				if ( colMajor )
					the_cmd.stmc.flags |= PADCMD_STRM_FLAG_CM;
				break;

			case PADCMD_SIM:
				the_cmd.simc.flags    = 0;
				the_cmd.simc.sdata[0] = the_cmd.simc.sdata[1] = 0;
				the_cmd.simc.a        = htonl(0xaaaa);
				the_cmd.simc.b        = htonl(0xbbbb);
				the_cmd.simc.c        = htonl(0xcccc);
				the_cmd.simc.d        = htonl(0xdddd);
				break;

			default:
				break;
		}
		client(sd, argv[optind], port, &the_cmd);
	}

	udpCommClose(sd);
	return 0;
}
