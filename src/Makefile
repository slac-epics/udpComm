TOP=..

include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
#LIBRARY_HOST        = udpCommBSD padSupport
LIBRARY_IOC_DEFAULT = udpCommBSD padSupport
LIBRARY_IOC_RTEMS   = udpCommBSD padSupport udpComm

PROD_HOST           = padProtoHost

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

padProtoHost_SRCS=padProtoHost.c bpmsim.c hostStream.c
padProtoHost_SRCS+=$(SDDS_$(USE_SDDS)_SRCS)
padProtoHost_LIBS=padSupport udpCommBSD
padProtoHost_LDLIBS+=$(SDDS_$(USE_SDDS)_LIBS)

INC += padProto.h padStream.h udpComm.h


#Indicate that we want to use 'normal' socket implementation
USR_CPPFLAGS += $(SDDS_$(USE_SDDS)_CPPFLAGS)

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
