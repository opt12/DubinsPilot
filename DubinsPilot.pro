######################################################################
# Automatically generated by qmake (2.01a) Mi. Apr 25 18:15:57 2018
######################################################################

TEMPLATE = app
TARGET = DubinsPilot
DEPENDPATH += . src Forms libXPlane
INCLUDEPATH += . src ./external/libXPlane ./external/json ./external/betterEnums ./external/qLedIndicator Forms

QT += widgets   # as per https://wiki.qt.io/Transition_from_Qt_4.x_to_Qt5

LIBS += -L/usr/lib/	#standard search path for libraries
LIBS += -lGeographic	#GeographicLib https://geographiclib.sourceforge.io/
#LIBS += -lpthread	#standard POSIX threading Library

#QWT Library: ATTENTION. QWT has a breaking interface change from version 6.0 to 6.1.
#Ensure to use maximum Version 6.0.2 of QWT Library
#Make sure to set the right search path to the QWT 6.0.2 Library
#INCLUDEPATH += /usr/include/qwt /usr/local/qwt-6.0.2/include	#Check for the correct path to QWT 6.0.2 Library headers
#LIBS += -L/usr/local/qwt-6.0.2/lib -lqwt		#QWT Library https://sourceforge.net/projects/qwt/files/qwt/6.0.2/; include search path if necessary
INCLUDEPATH += /usr/include/qwt 	
LIBS += -lqwt		#QWT Library https://sourceforge.net/projects/qwt; include search path if necessary

# TODO Check for Cygwin or whatever...
# see https://stackoverflow.com/questions/2952733/using-sys-socket-h-functions-on-windows

# Input
HEADERS += Forms/DubinsPilot_Dialog.h \
           src/DataCenter.h \
           src/Dataset.h \
           src/SegmentStatistics.h \
           src/pid_controller.h \
           src/Position.h \
           src/SockServer.h \
           src/RequestDispatcher.h \
           src/utils.h \
           src/DubinsPath.h \
           src/auto_pilot.h \
           src/FlightPhase.h \
           src/DubinsScheduler.h \
           src/ControlAutomation.h \
           src/constants.h
FORMS +=   Forms/DubinsPilot_Dialog.ui
SOURCES += Forms/DubinsPilot_Dialog.cpp \
           src/main.cpp \
           src/DataCenter.cpp \
           src/Dataset.cpp \
           src/SegmentStatistics.cpp \
           src/pid_controller.cpp \
           src/Position.cpp \
           src/SockServer.cpp \
           src/RequestDispatcher.cpp \
           src/utils.cpp \
           src/DubinsPath.cpp \
           src/auto_pilot.cpp \
           src/FlightPhase.cpp \
           src/ControlAutomation.cpp \
           src/DubinsScheduler.cpp

#branch of external libXPlane
HEADERS += ./external/libXPlane/XPlaneBeaconListener.h \
           ./external/libXPlane/XPlaneUDPClient.h \
           ./external/libXPlane/XPUtils.h
SOURCES += ./external/libXPlane/XPlaneBeaconListener.cpp \
           ./external/libXPlane/XPlaneUDPClient.cpp \
           ./external/libXPlane/XPUtils.cpp

#nlohman/json library
HEADERS += ./external/json/json.hpp

#betterEnums library
HEADERS += ./external/betterEnums/enum.h

#qLedIndicator library
HEADERS += ./external/qLedIndicator/qledindicator.h
SOURCES += ./external/qLedIndicator/qledindicator.cpp

DESTDIR =   ./target
OBJECTS_DIR = ./objDestination
MOC_DIR =   ./generated/moc
UI_DIR =    ./generated/ui
RCC_DIR =   ./generated/rc

#additional features and flags
QMAKE_CXXFLAGS += -std=c++11
QMAKE_CFLAGS += -std=c99
CONFIG+=debug	#comment this out for production build skipping Debug symbols
