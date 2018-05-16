#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=null
DEBUGGABLE_SUFFIX=null
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/test.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=Elf
DEBUGGABLE_SUFFIX=Elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/test.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=ble_main.c uart.c app.c automio.c bluetooth.c leds.c timers.c spi.c adc.c switches.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/ble_main.o ${OBJECTDIR}/uart.o ${OBJECTDIR}/app.o ${OBJECTDIR}/automio.o ${OBJECTDIR}/bluetooth.o ${OBJECTDIR}/leds.o ${OBJECTDIR}/timers.o ${OBJECTDIR}/spi.o ${OBJECTDIR}/adc.o ${OBJECTDIR}/switches.o
POSSIBLE_DEPFILES=${OBJECTDIR}/ble_main.o.d ${OBJECTDIR}/uart.o.d ${OBJECTDIR}/app.o.d ${OBJECTDIR}/automio.o.d ${OBJECTDIR}/bluetooth.o.d ${OBJECTDIR}/leds.o.d ${OBJECTDIR}/timers.o.d ${OBJECTDIR}/spi.o.d ${OBJECTDIR}/adc.o.d ${OBJECTDIR}/switches.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/ble_main.o ${OBJECTDIR}/uart.o ${OBJECTDIR}/app.o ${OBJECTDIR}/automio.o ${OBJECTDIR}/bluetooth.o ${OBJECTDIR}/leds.o ${OBJECTDIR}/timers.o ${OBJECTDIR}/spi.o ${OBJECTDIR}/adc.o ${OBJECTDIR}/switches.o

# Source Files
SOURCEFILES=ble_main.c uart.c app.c automio.c bluetooth.c leds.c timers.c spi.c adc.c switches.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/test.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/ble_main.o: ble_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ble_main.o.d 
	@${RM} ${OBJECTDIR}/ble_main.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  ble_main.c 
	
${OBJECTDIR}/uart.o: uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/uart.o.d 
	@${RM} ${OBJECTDIR}/uart.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  uart.c 
	
${OBJECTDIR}/app.o: app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/app.o.d 
	@${RM} ${OBJECTDIR}/app.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  app.c 
	
${OBJECTDIR}/automio.o: automio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/automio.o.d 
	@${RM} ${OBJECTDIR}/automio.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  automio.c 
	
${OBJECTDIR}/bluetooth.o: bluetooth.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/bluetooth.o.d 
	@${RM} ${OBJECTDIR}/bluetooth.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  bluetooth.c 
	
${OBJECTDIR}/leds.o: leds.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/leds.o.d 
	@${RM} ${OBJECTDIR}/leds.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  leds.c 
	
${OBJECTDIR}/timers.o: timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/timers.o.d 
	@${RM} ${OBJECTDIR}/timers.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  timers.c 
	
${OBJECTDIR}/spi.o: spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/spi.o.d 
	@${RM} ${OBJECTDIR}/spi.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  spi.c 
	
${OBJECTDIR}/adc.o: adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/adc.o.d 
	@${RM} ${OBJECTDIR}/adc.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  adc.c 
	
${OBJECTDIR}/switches.o: switches.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/switches.o.d 
	@${RM} ${OBJECTDIR}/switches.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  switches.c 
	
else
${OBJECTDIR}/ble_main.o: ble_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ble_main.o.d 
	@${RM} ${OBJECTDIR}/ble_main.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  ble_main.c 
	
${OBJECTDIR}/uart.o: uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/uart.o.d 
	@${RM} ${OBJECTDIR}/uart.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  uart.c 
	
${OBJECTDIR}/app.o: app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/app.o.d 
	@${RM} ${OBJECTDIR}/app.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  app.c 
	
${OBJECTDIR}/automio.o: automio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/automio.o.d 
	@${RM} ${OBJECTDIR}/automio.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  automio.c 
	
${OBJECTDIR}/bluetooth.o: bluetooth.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/bluetooth.o.d 
	@${RM} ${OBJECTDIR}/bluetooth.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  bluetooth.c 
	
${OBJECTDIR}/leds.o: leds.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/leds.o.d 
	@${RM} ${OBJECTDIR}/leds.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  leds.c 
	
${OBJECTDIR}/timers.o: timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/timers.o.d 
	@${RM} ${OBJECTDIR}/timers.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  timers.c 
	
${OBJECTDIR}/spi.o: spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/spi.o.d 
	@${RM} ${OBJECTDIR}/spi.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  spi.c 
	
${OBJECTDIR}/adc.o: adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/adc.o.d 
	@${RM} ${OBJECTDIR}/adc.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  adc.c 
	
${OBJECTDIR}/switches.o: switches.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/switches.o.d 
	@${RM} ${OBJECTDIR}/switches.o 
	 ${MP_CC} $(MP_EXTRA_CC_PRE)  switches.c 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/test.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
else
dist/${CND_CONF}/${IMAGE_TYPE}/test.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
