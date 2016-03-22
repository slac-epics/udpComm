TOP=..

include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
#LIBRARY_HOST        = udpCommBSD padSupport
LIBRARY_IOC_DEFAULT = padSupport padUdpCommSupport udpCommBSD 
LIBRARY_IOC_RTEMS   = padSupport padUdpCommSupport udpComm

PROD_HOST           = padProtoHost
PROD_IOC_RTEMS      = padStreamTestSrv
#PROD                = padStreamTestClnt

# SDDS_LIB is defined in the RELEASE file and points
# to the local SDDS installation
ifdef SDDS_LIB
USE_SDDS=YES
else
USE_SDDS=NO
endif

SDDS_YES_SRCS=sddsrd.c
SDDS_YES_LIBS=SDDS1c
SDDS_YES_CPPFLAGS=-DUSE_SDDS

TEST_ONLY_YES_CPPFLAGS=-DTEST_ONLY

padProtoHost_SRCS=padProtoHost.c bpmsim.c hostStream.c
padProtoHost_SRCS+=$(SDDS_$(USE_SDDS)_SRCS)
padProtoHost_LIBS=padSupport udpCommBSD
padProtoHost_LDLIBS+=$(SDDS_$(USE_SDDS)_LIBS)

INC += padProto.h padStream.h udpComm.h
INC += fidProcess.h drvPadUdpComm.h drvPadUdpCommIO.h wavBuf.h
INC += drvPadAdcStream.h


#Indicate that we want to use 'normal' socket implementation
USR_CPPFLAGS += $(SDDS_$(USE_SDDS)_CPPFLAGS) $(TEST_ONLY_$(TEST_ONLY)_CPPFLAGS)

# install devXxxSoft.dbd into <top>/dbd
DBD += padUdpCommSupport.dbd
DBD += padStreamTestSrv.dbd
DBD += padStreamTestClnt.dbd

# The following are compiled and added to the Support libraries
padSupport_SRCS += padProto.c

padUdpCommSupport_SRCS += drvPadUdpComm.c fidProcess.c wavBuf.c devWfRawSig.c hwtime.c

padSupport_SRCS_RTEMS-uC5282 = padStream.c bpmsim.c drvPadAdcStream.c

padSupport_SRCS += $(padSupport_SRCS_$(T_A))


udpCommBSD_SRCS += udpCommBSD.c
udpComm_SRCS    += udpComm.c

padStreamTestSrv_SRCS += padStreamTestSrv_registerRecordDeviceDriver.cpp
padStreamTestSrv_SRCS += drvPadStreamTestSrv.c
padStreamTestSrv_LIBS += padSupport udpComm $(EPICS_BASE_IOC_LIBS)
padStreamTestSrv_DBD  += base.dbd
padStreamTestSrv_DBD  += drvPadStreamTestSrv.dbd

padStreamTestClnt_SRCS_DEFAULT += padStreamTestClntMain.c
padStreamTestClnt_SRCS_RTEMS   += -nil-
padStreamTestClnt_SRCS += padStreamTestClnt_registerRecordDeviceDriver.cpp
padStreamTestClnt_SRCS += padStreamTestClnt.c
padStreamTestClnt_LIBS_DEFAULT = udpCommBSD
padStreamTestClnt_LIBS_RTEMS   = udpComm
padStreamTestClnt_LIBS += padUdpCommSupport padSupport $(EPICS_BASE_IOC_LIBS)
padStreamTestClnt_DBD  += base.dbd drvPadStreamTestClnt.dbd padUdpCommSupport.dbd

#===========================

include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE

