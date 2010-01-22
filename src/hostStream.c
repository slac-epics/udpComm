#include <udpComm.h>
#include <padProto.h>
#include <padStream.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <netinet/in.h>

#include <math.h>

#define NCHNS 4

#include "bpmsim.h"
#include "hostStream.h"

static time_t      petTime;
time_t             padStreamTimeoutSecs = 10;

#ifdef USE_SDDS
#include "sddsrd.h"
#include <string.h>

static char       *sddsStr         =  0;
static char       *sddsFnam        =  0;
static char       *sddsCols[NCOLS] = {0};
static SddsFileDat sddsDat         =  0;
static SddsPage    sddsPag         =  0;
static int         sddsPgFst       =  0;
static int         sddsPgLst       = -1;

static void
clearNames()
{
int i;
	free(sddsStr);
	sddsStr  = 0;

	sddsFnam = 0;

	for ( i=0; i < sizeof(sddsCols)/sizeof(sddsCols[0]); i++ )
		sddsCols[i] = 0;
}

/* 'fspec' is of the form  filename ':' [col] [ ',' [col] ] */

int
padStreamSddsSetup(const char *fspec, int pFst, int pLst)
{
int ncols;
char *colp;

	/* clear leftovers from last time */
	clearNames();

	if ( !fspec || ! ( (sddsStr = strdup(fspec)), (colp = strchr(sddsStr,':'))) ) {
		fprintf(stderr,"padStreamSddsSetup: file specifier must be of the form <file_name>:[<column_name>][,[<column_name>]]...\n");
		clearNames();
		return -1;
	}

	sddsFnam  = sddsStr;
	*colp     = 0;
	colp++;

	ncols     =  0;

	sddsPgFst = pFst;
	sddsPgLst = pLst;

	while ( colp && *colp && ncols < sizeof(sddsCols)/sizeof(sddsCols[0]) ) {
		sddsCols[ncols++]= ',' == *colp ? 0 : colp;
		if ( (colp = strchr(colp,',')) ) {
			*colp = 0;
			colp++;
		}
	}

	if ( ! ncols ) {
		fprintf(stderr,"padStreamSddsSetup: no columns given!\n");
		clearNames();
		return -1;
	}
	return 0;
}
#endif

static int bigEndian()
{
union {
	uint8_t  x[2];
	uint16_t tst;
} endian = { x : {0xBE, 0} };
	return(endian.tst == 0xBE00);
}

typedef struct StripSimValRec_ {
	int32_t	a,b,c,d;
} StripSimValRec, *StripSimVal;

static __inline__ int32_t swapl(int32_t x)
{
uint32_t v = (uint32_t)x;
	v = ((v & 0x00ff00ff) << 8) | ((v>>8) & 0x00ff00ff);
	v = (v >> 16) | (v<<16);
	return (int32_t) v;
}

static void *
streamSim(void *packetBuffer,
			int nsamples,
			int d32,
			int little_endian,
			int column_major,
			void *uarg)
{
int16_t		*buf  = packetBuffer;
int32_t     *bufl = packetBuffer;
StripSimVal ini   = uarg;
int         swp;
static unsigned long noise = 1;
float       c,s;
int         ai,bi,ci,di;
int         aq,bq,cq,dq;
int32_t     atmp, btmp, ctmp, dtmp;
int         i;

	swp    = ( bigEndian() != !little_endian );

	if ( d32 ) {
		if ( swp ) {
			atmp = swapl(ini->a);
			btmp = swapl(ini->b);
			ctmp = swapl(ini->c);
			dtmp = swapl(ini->d);
		} else {
			atmp = ini->a;
			btmp = ini->b;
			ctmp = ini->c;
			dtmp = ini->d;
		}
		if ( column_major ) {
			for ( i=0; i<nsamples; i++ ) {
				bufl[i + 0] = atmp;
				bufl[i + 1] = btmp;
				bufl[i + 2] = ctmp;
				bufl[i + 3] = dtmp;
			}
		} else {
			for ( i=0; i<nsamples; i++ ) {
				bufl[i + 0*nsamples] = atmp;
				bufl[i + 1*nsamples] = btmp;
				bufl[i + 2*nsamples] = ctmp;
				bufl[i + 3*nsamples] = dtmp;
			}
		}
	} else {

		/* We can introduce random phase shift and time of arrival
		 * (common-mode to all channels)
		 * by submitting the first two samples of the filter
		 * recursion.
		 *
		 * Since  A * f(n) + B * f(n-1) o-@  A * F(z) + B * 1/z * F(z) 
		 *
		 * and we want to maintain the signal energy constant:
		 *
		 *  / (A+B/z)(A+B/conj(z)) |F(z)|^2 df == independent of A/B ratio
		 *
		 *  -> A^2 + B^2 / (|z|^2) + AB (1/z + conj(1/z))
		 *
		 *  should be constant as we vary A/B; 
		 *
		 *  |z|^2 = 1 on unit circle and we neglect the  1/z+conj(1/z) term
		 *  assuming that
		 *
		 *   / |F(z)|^2 cos(2pi f Ts) df is very small.
		 *
		 *  => A^2 + B^2 == const, hence we can set
		 *
		 *  A = X cos(phi)
		 *  B = X sin(phi)
		 *
		 *  for given amplitude X and a random phase 'phi'
		 */

		c = cosf(2*3.14159265*randval(noise)/(float)RANDVALMAX);
		s = sinf(2*3.14159265*randval(noise)/(float)RANDVALMAX);

		/*
		   c=1.; s=0.;
		   if ( c > 0. ) {
		   c = 1.; s = 0.;
		   } else {
		   c = 0.; s = 1.;
		   }
		 */

		randstep(&noise);

		ai = (int)((float)ini->a * c); aq	= (int)((float)ini->a * s);
		bi = (int)((float)ini->b * c); bq	= (int)((float)ini->b * s);
		ci = (int)((float)ini->c * c); cq	= (int)((float)ini->c * s);
		di = (int)((float)ini->d * c); dq	= (int)((float)ini->d * s);

		if ( column_major ) {
			iir2_bpmsim(buf++, nsamples, ai, aq,  &noise, swp, NCHNS);
			iir2_bpmsim(buf++, nsamples, bi, bq,  &noise, swp, NCHNS);
			iir2_bpmsim(buf++, nsamples, ci, cq,  &noise, swp, NCHNS);
			iir2_bpmsim(buf++, nsamples, di, dq,  &noise, swp, NCHNS);
		} else {
			iir2_bpmsim(buf, nsamples, ai, aq, &noise, swp, 1);
			buf += nsamples;
			iir2_bpmsim(buf, nsamples, bi, bq, &noise, swp, 1);
			buf += nsamples;
			iir2_bpmsim(buf, nsamples, ci, cq, &noise, swp, 1);
			buf += nsamples;
			iir2_bpmsim(buf, nsamples, di, dq, &noise, swp, 1);
		}

	}

	return packetBuffer;
}

static int
strmReplySetup(PadReply rply, uint32_t xid, int d32, int le, int col_maj, int nsamples, int chnl)
{
int len = nsamples*(d32 ? sizeof(int32_t) : sizeof(int16_t))*NCHNS + sizeof(*rply);

	/* Setup Reply */
	rply->version         = PADPROTO_VERSION2;
	rply->type            = PADCMD_STRM | PADCMD_RPLY;
	rply->chnl            = chnl;
	rply->nBytes          = htons(len);
	rply->timestampHi     = htons(0);
	rply->timestampLo     = htonl(0);
	rply->xid             = xid;
	rply->stat            = 0;
	rply->strm_cmd_flags  = PADRPLY_STRM_FLAG_TYPE_SET(0);
	if ( le )
		rply->strm_cmd_flags |= PADCMD_STRM_FLAG_LE;
	if ( col_maj )
		rply->strm_cmd_flags |= PADCMD_STRM_FLAG_CM;
	if ( d32 )
		rply->strm_cmd_flags |= PADCMD_STRM_FLAG_32;
	rply->strm_cmd_idx    = 0;

	return len;
}

/* refresh timestamp and transaction id */
static void
dopet(PadRequest req, PadReply rply)
{
struct timeval now;
	rply->timestampHi = req->timestampHi;
	rply->timestampLo = req->timestampLo;
	rply->xid         = req->xid;
	gettimeofday(&now, 0);
	petTime           = now.tv_sec;
}

static int sd   = -1;
PadReply   rply =  0 ;

int
padStreamPet(PadRequest req, uint32_t hostip)
{
	if ( sd < 0 )
		return -ENOTCONN;
	/* FIXME should verify that hostip == connected peer's address here...*/
	dopet(req, rply);
	return 0;
}

int
padStreamStart(PadRequest req, PadStrmCommand cmd, int me, uint32_t hostip)
{
uint32_t nsamples = ntohl(cmd->nsamples);
int      le       = cmd->flags & PADCMD_STRM_FLAG_LE;
int      cm       = cmd->flags & PADCMD_STRM_FLAG_CM;
int      d32      = cmd->flags & PADCMD_STRM_FLAG_32;

#if 0
	if ( sd >= 0 )
		udpCommClose(sd);
#else
	if ( sd < 0 ) {
#endif

	if ( (sd = udpCommSocket(6668)) < 0 )
		goto bail;

	if ( udpCommConnect(sd, hostip, ntohs(cmd->port)) ) {
		udpCommClose(sd);
		sd = -1;
		goto bail;
	}

#if 1
	}
#endif

#ifdef USE_SDDS
	if ( sddsFnam ) {
		if ( sddsDat ) {
			if (  nsamples != sddsDat->pages->nSamples
				|| ((d32? FLG_32:0) ^ (sddsDat->pages->flags & FLG_32))
				|| ((le ? FLG_LE:0) ^ (sddsDat->pages->flags & FLG_LE))
				|| ((cm ? FLG_CM:0) ^ (sddsDat->pages->flags & FLG_CM) )) {
				fprintf(stderr,"Error: cannot restart SDDS stream with different layout, endianness or sample number\n");
				return -1;
			}
		} else {
			if ( ! ( sddsDat = sddsFileSlurp(
							sddsFnam,
							sddsCols[0], sddsCols[1], 
							sddsCols[2], sddsCols[3], 
							sddsPgFst, sddsPgLst,
							nsamples,
							(d32? FLG_32 : 0) |
							(cm ? FLG_CM : 0) |
							(le ? FLG_LE : 0)
							) ) ) {
				fprintf(stderr,"Unable to read SDDS file\n");
				goto bail;
			}
		}
	}
#endif

	if ( !rply )
		rply = malloc(2048);

	strmReplySetup(rply, req ? req->xid : 0, d32, le, cm, nsamples, me);

	dopet(req, rply);

	return 0;

bail:
	perror("padStreamStart failed");
	return errno ? -errno : -1;
}


int
padStream(int32_t a, int32_t b, int32_t c, int32_t d)
{
StripSimValRec      v = {a,b,c,d};
int                 len, nsamples, d32;
struct timeval      now;

	if ( sd < 0 )
		return -ENODEV;

	gettimeofday(&now, 0);
	if ( now.tv_sec - petTime > padStreamTimeoutSecs ) {
		padStreamStop(0);
		return -ETIMEDOUT;
	}

	len      = ntohs(rply->nBytes);

#ifdef USE_SDDS
	if ( sddsDat ) {
		if ( ! sddsPag )
			sddsPag = sddsDat->pages;

		memcpy(rply->data, sddsPag->data.r, len - sizeof(*rply));

		sddsPag = sddsPag->next;

	} else {
#endif
	d32 = rply->strm_cmd_flags & PADCMD_STRM_FLAG_32;
	nsamples = (len - sizeof(*rply))/NCHNS/(d32 ? sizeof(int32_t) : sizeof(int16_t));

	streamSim(
		rply->data,
		nsamples, 
		d32,
		rply->strm_cmd_flags & PADCMD_STRM_FLAG_LE,
		rply->strm_cmd_flags & PADCMD_STRM_FLAG_CM,
		&v);
#ifdef USE_SDDS
	}
#endif

	udpCommSend(sd, rply, len);

	return 0;
}

static int32_t a = 2000;
static int32_t b = 3000;
static int32_t c = 4000;
static int32_t d = 5000;

int
padStreamSimulated(void *unused)
{
	padStream(a,b,c,d);
	return 0;
}

int
padStreamSim(PadSimCommand scmd, uint32_t hostip)
{
int dosend = 1;


	if ( scmd ) {
		dosend =  ! (PADCMD_SIM_FLAG_NOSEND & scmd->flags );

		if ( dosend ) {
			if ( sd < 0 )
				return -ENOTCONN; /* stream not started */
			/* FIXME: should verify hostip against connected host here */
		}

		a = ntohl(scmd->a);
		b = ntohl(scmd->b);
		c = ntohl(scmd->c);
		d = ntohl(scmd->d);
	}

	return  dosend ? padStream(a,b,c,d) : 0;
}

int
padStreamStop(uint32_t hostip)
{
	if ( sd < 0 ) 
		return -ENOTCONN;
	/* FIXME: should verify against connected host here */
	udpCommClose(sd);
	sd = -1;
#ifdef USE_SDDS
	sddsDatClean(sddsDat);
	sddsDat = 0;
	sddsPag = 0;
#endif
	return 0;
}


#ifdef MAIN
int main()
{
PadStrmCommandRec cmd;
int               chan = 0;

	cmd.type     = PADCMD_STRM;
	cmd.flags    = PADCMD_STRM_FLAG_LE | PADCMD_STRM_FLAG_CM; 
	cmd.nsamples = htonl(128);
	cmd.port     = htons(PADPROTO_STRM_PORT);

	padStreamStart(0, &cmd, chan, inet_addr("127.0.0.1"));

	padStream(1000,2000,3000,4000);
}
#endif
