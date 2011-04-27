#include <drvPadUdpComm.h>
#include <padProto.h>
#include <errlog.h>
#include <registry.h>
#include <epicsExport.h>

DrvPadUdpCommPrefsRec     padStreamTestClntPrefs = {
	nsamples:              128,
	d32:                     0,
	nsamples_dynamic:        1,
	nchannels_dynamic:       1,
	col_major:               1
};


static int
cook(PadReply rply, int nsamples, int layout_cm, int kind, void *closure)
{
	epicsPrintf("Kind: %i, Index: %s - %u, nsamples: %4i, layout: %cM, FLAGS: %s|%s|%s\n",
		kind,
		(rply->strm_cmd_idx & PADRPLY_STRM_CMD_IDX_MF) ? "(MF)":"    ",
		PADRPLY_STRM_CMD_IDX_GET(rply->strm_cmd_idx),
		nsamples,
		layout_cm ? 'C':'R',
		(rply->strm_cmd_flags & PADCMD_STRM_FLAG_C1) ? "C1" :"C4",
		(rply->strm_cmd_flags & PADCMD_STRM_FLAG_32) ? "32" :"16",
		(rply->strm_cmd_flags & PADCMD_STRM_FLAG_LE) ? "LE" :"BE"
	);
	return PAD_UDPCOMM_COOK_STAT_OK;
}

static int
get_prefs(DrvPadUdpCommPrefs p)
{
	*p = padStreamTestClntPrefs;
	return 0;
}


DrvPadUdpCommCallbacksRec padStreamTestClntCbs = {
	init:                  0,
	cookClosure:           0,
	cook:                  cook,
	watchdogClosure:       0,
	watchdog:              0,
	watchKinds:            (1<<0),
	get_prefs:             get_prefs,
	sim_pos:               0,
	fid_process_install:   0
};

static void
padStreamTestClntRegistrar(void)
{
	registryAdd( (void*)drvPadUdpCommRegistryId, "padStreamTestClntCbs", &padStreamTestClntCbs );
}

epicsExportRegistrar(padStreamTestClntRegistrar);
