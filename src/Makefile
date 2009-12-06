TOP=..
include $(TOP)/configure/CONFIG

CROSS_COMPILER_TARGET_ARCHS=RTEMS-uC5282 RTEMS-beatnik

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
LIBRARY_HOST      = udpCommBSD padSupport
LIBRARY_IOC_RTEMS = udpCommBSD padSupport udpComm

PROD_HOST         = padProtoHost

USE_SDDS=YES

SDDS_YES_SRCS=sddsrd.c
# a hack because my SDDS build used another BASE version
# and the paranoia check wouldn't let me use it
ifeq (hack,hack)
SDDS_YES_LIBS=SDDS1c
else
SDDS_YES_LDFLAGS=-L/home/till/epics/extensions/lib/$(T_A)
USR_LDFLAGS+=$(SDDS_$(USE_SDDS)_LDFLAGS)
SDDS_YES_LIBS=SDDS1c
endif
SDDS_YES_CPPFLAGS=-DUSE_SDDS

padProtoHost_SRCS=padProtoHost.c bpmsim.c hostStream.c
padProtoHost_SRCS+=$(SDDS_$(USE_SDDS)_SRCS)
padProtoHost_LIBS=padSupport udpCommBSD
padProtoHost_LDLIBS+=$(SDDS_$(USE_SDDS)_LIBS)

INC += padProto.h padStream.h udpComm.h


#Indicate that we want to use 'normal' socket implementation
USR_CPPFLAGS += -DBSDSOCKET $(SDDS_$(USE_SDDS)_CPPFLAGS)

# install devXxxSoft.dbd into <top>/dbd
#DBD += bpmPadSupport.dbd

# The following are compiled and added to the Support libraries
padSupport_SRCS += padProto.c

padSupport_SRCS_RTEMS-uC5282 = padStream.c bpmsim.c

padSupport_SRCS += $(padSupport_SRCS_$(T_A))


udpCommBSD_SRCS += udpCommBSD.c
udpComm_SRCS    += udpComm.c

#===========================

include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE
