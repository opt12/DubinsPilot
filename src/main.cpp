/*
 * main.cpp
 *
 *  Created on: 25.04.2018
 *      Author: eckstein
 */
#include <DataCenter.h>

#include <QtGui>
#include <qobject.h>
#include <iostream>

#include "DubinsPilot_Dialog.h"
#include "SockServer.h"
#include "RequestDispatcher.h"
#include "auto_pilot.h"
#include "DubinsPath.h"
#include "enumDeclarations.h"
#include "DubinsScheduler.h"
#include "ControlAutomation.h"
#include "Gym.h"

#include "json.hpp"
// for convenience
using json = nlohmann::json;

#include <qmetatype.h>

class DubinsPilotDialog;


int main(int argc, char *argv[]) {

	QApplication app(argc, argv);

	qRegisterMetaType<json>("json");

	AutoPilot ap;	//have this before the DubinsPilotDialog as we read in the controller settings there

	DubinsScheduler dubSched;

	DubinsPilotDialog *mainDialog;
	mainDialog = new DubinsPilotDialog;
	mainDialog->setWindowTitle(QObject::tr("DubinsPilot - an Autopilot for Emergency Landings"));

	mainDialog->show();

	DataCenter *dc = DataCenter::getInstance();

	SockServer *sock = SockServer::getInstance();
	// TODO remove debug mode for socket
//	sock->setDebug(true);

	RequestDispatcher rqd;

	Gym gym;

	ControlAutomation ctrlAuto;


	//connections from DubinsPilotDialog --> AutoPilot
	QObject::connect(mainDialog,
			SIGNAL(sigPidParametersChanged(ctrlType, double, double, double)),
			&ap,
			SLOT(setControllerParameters(ctrlType, double, double, double)));
	QObject::connect(mainDialog,
			SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)), &ap,
			SLOT(invokeController(ctrlType, bool)));
	QObject::connect(mainDialog,
			SIGNAL(sigRequestedSetValueChanged(ctrlType, double)), &ap,
			SLOT(requestCtrlTargetValue(ctrlType, double)));
	QObject::connect(mainDialog,
			SIGNAL(sigCircleDirectionChanged(bool, double)), &ap,
			SLOT(requestCircleDirection(bool, double)));

	//connections from DubinsScheduler --> AutoPilot
	QObject::connect(&dubSched,
			SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)), &ap,
			SLOT(invokeController(ctrlType, bool)));
	QObject::connect(&dubSched,
			SIGNAL(sigRequestedSetValueChanged(ctrlType, double, bool, bool)), &ap,
			SLOT(requestCtrlTargetValue(ctrlType, double, bool, bool)));
	QObject::connect(&dubSched,
			SIGNAL(sigCircleDirectionChanged(bool, double)), &ap,
			SLOT(requestCircleDirection(bool, double)));

	//connections from DubinsPilotDialog --> DataCenter
	QObject::connect(mainDialog,
			SIGNAL(sigLoggingActiveStateChanged(bool, QDir, QString)), dc,
			SLOT(invokeLogging(bool, QDir, QString)));
	QObject::connect(mainDialog, SIGNAL(sigSendXPDataRef(const char*, double)),
			dc, SLOT(SendXPDataRef(const char*, double)));
	QObject::connect(mainDialog,
			SIGNAL(sigSetElfLocation(double, double, double, double, pathTypeEnum)), dc,
			SLOT(setElfLocation(double, double, double, double, pathTypeEnum)));
	QObject::connect(mainDialog,
			SIGNAL(sigSetElfLocation(Position_WGS84, double, pathTypeEnum)), dc,
			SLOT(setElfLocation(Position_WGS84, double, pathTypeEnum)));
	QObject::connect(mainDialog,
			SIGNAL(sigResetElf(void)), dc,
			SLOT(resetElfLocation(void)));
	QObject::connect(mainDialog,
				SIGNAL(sigSetSimulationPaused(bool)), dc,
				SLOT(setIsSimulationPaused(bool)));
	QObject::connect(mainDialog,
				SIGNAL(sigFlightPathCharacteristicsChanged(double, double, double)), dc,
				SLOT(setFlightPathCharacteristics(double, double, double)));


	//connections from AutoPilot --> DataCenter
	QObject::connect(&ap, SIGNAL(sigSendXPDataRef(const char*, double)), dc,
			SLOT(SendXPDataRef(const char*, double)));
	QObject::connect(&ap, SIGNAL(sigSendXPDataRef(const char*, bool)), dc,
			SLOT(SendXPDataRef(const char*, bool)));

	//connections from DataCenter --> DubinsPilotDialog
	QObject::connect(dc, SIGNAL(XPlaneConnectionDisplayUI(bool)), mainDialog,
			SLOT(displayXPlaneConnection(bool)));
	QObject::connect(dc, SIGNAL(sigElfCoordsSet(Position_WGS84, double, bool)), mainDialog,
			SLOT(showElfCoords(Position_WGS84, double, bool)));
	QObject::connect(dc, SIGNAL(sigWindChanged(double, double)), mainDialog,
			SLOT(displayCurrentWind(double, double)));
	QObject::connect(dc, SIGNAL(sigLoggingStateChanged(bool)), mainDialog,
			SLOT(setLoggingState(bool)));

	//connections from AutoPilot --> DubinsPilotDialog
	QObject::connect(&ap,
			SIGNAL(sigAttachControllerCurve(ctrlType, QwtPlotCurve*)),
			mainDialog, SLOT(attachControllerCurve(ctrlType, QwtPlotCurve*)));
	QObject::connect(&ap, SIGNAL(sigReplotControllerCurve(ctrlType)), mainDialog,
			SLOT(replotControllerCurve(ctrlType)));
	QObject::connect(&ap, SIGNAL(sigRequestTargetValue(ctrlType, double, bool)), mainDialog,
			SLOT(setTargetValueControlKnob(ctrlType, double, bool)));
	QObject::connect(&ap,
			SIGNAL(sigSetControllerCheckButtons(ctrlType, bool, bool)),
			mainDialog, SLOT(setControllerCheckButtons(ctrlType, bool, bool)));

	//connections from SockServer --> RequestDispatcher
	QObject::connect(sock, SIGNAL(sigDispatchSockMessage(json)), &rqd,
			SLOT(dispatchSockMessage(json)));

	//connections from RequestDispatcher --> Gym
	QObject::connect(&rqd, SIGNAL(sigSetPlaneState(json, int)), &gym, SLOT(setPlaneState(json, int)));
	QObject::connect(&rqd, SIGNAL(sigSendGymControl(const char*, double)), &gym, SLOT(setGymControl(const char*, double)));

	//connections from Gym --> DataCenter
	QObject::connect(&gym, SIGNAL(sigSendXPDataRef(const char*, double)), dc, SLOT(SendXPDataRef(const char*, double)));


	//connections from DataCenter --> SockServer
	QObject::connect(dc, SIGNAL(sigSocketSendData(std::string, int, json)),
			sock, SLOT(socketSendData(std::string, int, json)));
	//connections from AutoPilot --> SockServer
	QObject::connect(&ap, SIGNAL(sigSocketSendData(std::string, int, json)),
			sock, SLOT(socketSendData(std::string, int, json)));

	//connections from DubinsPilotDialog --> DubinsScheduler
	QObject::connect(mainDialog, SIGNAL(sigStartPathTracking(bool)),
			&dubSched, SLOT(takeMeDown(bool)));

	//connections from DubinsScheduler --> DubinsPilotDialog
	QObject::connect(&dubSched, SIGNAL(sigDisplayFlightPhase(QString, QString)),
				mainDialog, SLOT(displayFlightPhase(QString, QString)));
	QObject::connect(&dubSched, SIGNAL(sigPathTrackingStatus(bool)),
				mainDialog, SLOT(displayPathTrackingStatus(bool)));
	QObject::connect(&dubSched, SIGNAL(sigPauseSimTriggered(bool)),
			mainDialog, SLOT(clickPause(bool)));

	//connections from DubinsScheduler --> DataCenter
	QObject::connect(&dubSched, SIGNAL(sigOutputPathTrackingStats(const json)),
				dc, SLOT(outputPathTrackingStats(const json)));

	//connections from DubinsScheduler --> SockServer
	QObject::connect(&dubSched, SIGNAL(sigSocketSendData(std::string, int, json)),
			sock, SLOT(socketSendData(std::string, int, json)));


	//connections from DubinsPilotDialog --> ControlAutomation
	QObject::connect(mainDialog, SIGNAL(sigLifeSaverHeightChanged(QString)),
			&ctrlAuto, SLOT(setLifeSaverHeightChanged(QString)));
	QObject::connect(mainDialog, SIGNAL(sigApproachStartingAltitudeChanged(QString)),
			&ctrlAuto, SLOT(setApproachStartingAltitudeChanged(QString)));

	//connections from ControlAutomation --> DubinsPilotDialog
	QObject::connect(&ctrlAuto, SIGNAL(sigLifeSaverTriggered(void)),
			mainDialog, SLOT(clickSetHeight(void)));
	QObject::connect(&ctrlAuto, SIGNAL(sigApproachStartingTriggered(void)),
			mainDialog, SLOT(clickTakeMeDown(void)));



	return app.exec();
}

