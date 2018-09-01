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

#include "pid_parameters.h"
#include "SockServer.h"
#include "RequestDispatcher.h"
#include "auto_pilot.h"
#include "DubinsPath.h"
#include "enumDeclarations.h"
#include "DubinsScheduler.h"
#include "ControlAutomation.h"

#include "json.hpp"
// for convenience
using json = nlohmann::json;

#include <qmetatype.h>

//#define TEST

int main(int argc, char *argv[]) {
#ifdef TEST
#include "Position.h"
#include "enumDeclarations.h"
	Position_Cartesian startCart = Position_Cartesian(1500.0, 300.0, 1000.0),
			endCart = Position_Cartesian(500.0, 300.0, -200.0);
	Position start, end;
	start = Position(Position::getOrigin_WGS84(), startCart.x, startCart.y, startCart.z);
	end = Position(Position::getOrigin_WGS84(), endCart.x, endCart.y, endCart.z);
	double startHeading = 135;
	double endHeading = 180;
	double _circleRadius = 500;
	double _circleGlideRatio = 10;
	double _straightGlideRatio = 10;
	pathTypeEnum pathType = pathTypeEnum::LSL;
	DubinsPath db(start, end,
		startHeading, endHeading, _circleRadius,
		_circleGlideRatio, _straightGlideRatio,
		 pathType);
	std::cout << "shortest DubinsPath "<< pathType << " in Matlab is given as:\n";
	std::cout << db.getPathAsMatlabCommand() << std::endl;;
	std::cout << "extended DubinsPath "<< pathType << " in Matlab is given as:\n";
	std::cout << db.getExtendedPathAsMatlabCommand() << std::endl;;

	pathType = pathTypeEnum::RSR;
	db = DubinsPath(start, end,
		startHeading, endHeading, _circleRadius,
		_circleGlideRatio, _straightGlideRatio,
		pathType);

	std::cout << "DubinsPath "<< pathType << " in Matlab is given as:\n";
	std::cout << db.getPathAsMatlabCommand() << std::endl;;
	exit(0);
#else

	QApplication app(argc, argv);

	qRegisterMetaType<json>("json");

	AutoPilot ap;	//have this before the PIDParametersDialog as we read in the controller settings there

	DubinsScheduler dubSched;

	PIDParametersDialog *pidParams;
	pidParams = new PIDParametersDialog;
	pidParams->setWindowTitle(QObject::tr("PID Parameter Selector"));

	pidParams->show();

	DataCenter *dc = DataCenter::getInstance();

	SockServer *sock = SockServer::getInstance();
	// TODO remove debug mode for socket
//	sock->setDebug(true);

	RequestDispatcher rqd;

	ControlAutomation ctrlAuto;


	//connections from PIDParametersDialog --> AutoPilot
	QObject::connect(pidParams,
			SIGNAL(sigPidParametersChanged(ctrlType, double, double, double)),
			&ap,
			SLOT(setControllerParameters(ctrlType, double, double, double)));
	QObject::connect(pidParams,
			SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)), &ap,
			SLOT(invokeController(ctrlType, bool)));
	QObject::connect(pidParams,
			SIGNAL(sigRequestedSetValueChanged(ctrlType, double)), &ap,
			SLOT(requestCtrlTargetValue(ctrlType, double)));
	QObject::connect(pidParams,
			SIGNAL(sigCircleDirectionChanged(bool, double)), &ap,
			SLOT(requestCircleDirection(bool, double)));

	//connections from PIDParametersDialog --> DataCenter
	QObject::connect(pidParams,
			SIGNAL(sigLoggingActiveStateChanged(bool, QFile *)), dc,
			SLOT(invokeLogging(bool, QFile *)));
	QObject::connect(pidParams, SIGNAL(resetOrigin(void)), dc,
			SLOT(setOrigin(void)));
	QObject::connect(pidParams, SIGNAL(sigSendXPDataRef(const char*, double)),
			dc, SLOT(SendXPDataRef(const char*, double)));
	QObject::connect(pidParams,
			SIGNAL(sigSetElfLocation(double, double, double, double, pathTypeEnum)), dc,
			SLOT(setElfLocation(double, double, double, double, pathTypeEnum)));
	QObject::connect(pidParams,
			SIGNAL(sigSetElfLocation(Position_WGS84, double, pathTypeEnum)), dc,
			SLOT(setElfLocation(Position_WGS84, double, pathTypeEnum)));
	QObject::connect(pidParams,
			SIGNAL(sigResetElf(void)), dc,
			SLOT(resetElfLocation(void)));
	QObject::connect(pidParams,
				SIGNAL(sigSetSimulationPaused(bool)), dc,
				SLOT(setIsSimulationPaused(bool)));
	QObject::connect(pidParams,
				SIGNAL(sigFlightPathCharacteristicsChanged(double, double, double)), dc,
				SLOT(setFlightPathCharacteristics(double, double, double)));


	//connections from AutoPilot --> DataCenter
	QObject::connect(&ap, SIGNAL(sigSendXPDataRef(const char*, double)), dc,
			SLOT(SendXPDataRef(const char*, double)));
	QObject::connect(&ap, SIGNAL(sigSendXPDataRef(const char*, bool)), dc,
			SLOT(SendXPDataRef(const char*, bool)));

	//connections from DataCenter --> PIDParametersDialog
	QObject::connect(dc, SIGNAL(XPlaneConnectionChanged(bool)), pidParams,
			SLOT(setXPlaneConnection(bool)));
//	QObject::connect(dc, SIGNAL(originSetTo(Position_WGS84)), pidParams,
//			SLOT(showOriginCoords(Position_WGS84)));
	QObject::connect(dc, SIGNAL(sigElfCoordsSet(Position_WGS84, double, bool)), pidParams,
			SLOT(showElfCoords(Position_WGS84, double, bool)));
	QObject::connect(dc, SIGNAL(sigWindChanged(double, double)), pidParams,
			SLOT(displayCurrentWind(double, double)));


	//connections from AutoPilot --> PIDParametersDialog
	QObject::connect(&ap,
			SIGNAL(sigAttachControllerCurve(ctrlType, QwtPlotCurve*)),
			pidParams, SLOT(attachControllerCurve(ctrlType, QwtPlotCurve*)));
	QObject::connect(&ap, SIGNAL(sigReplotControllerCurve(ctrlType)), pidParams,
			SLOT(replotControllerCurve(ctrlType)));

	QObject::connect(&ap, SIGNAL(sigRequestTargetValue(ctrlType, double, bool)), pidParams,
			SLOT(setTargetValueControlKnob(ctrlType, double, bool)));

	QObject::connect(&ap,
			SIGNAL(sigSetControllerCheckButtons(ctrlType, bool, bool)),
			pidParams, SLOT(setControllerCheckButtons(ctrlType, bool, bool)));

	std::cout<< "Connected AutoPilot --> PIDParametersDialog\n";


	//connections from SockServer --> RequestDispatcher
	QObject::connect(sock, SIGNAL(sigDispatchSockMessage(json)), &rqd,
			SLOT(dispatchSockMessage(json)));

//TODO Prüfe, ob das alles nicht einfach rauskann, weil ich es aktuell nicht benutze
	//connections from RequestDispatcher --> DataCenter
//	QObject::connect(&rqd, SIGNAL(requested_getPosition(int)), dc,
//			SLOT(on_rqd_requested_getPosition(int)));
//	QObject::connect(&rqd, SIGNAL(requested_getPositionXY(int)), dc,
//			SLOT(on_rpd_requested_getPositionXY(int)));
//	QObject::connect(&rqd, SIGNAL(requested_getOrigin(int)), dc,
//			SLOT(on_rqd_requested_getOrigin(int)));
//	QObject::connect(&rqd, SIGNAL(requested_getPlaneState(int)), dc,
//			SLOT(on_rqd_requested_getPlaneState(int)));

	//connections from DataCenter --> SockServer
	QObject::connect(dc, SIGNAL(sigSocketSendData(std::string, int, json)),
			sock, SLOT(socketSendData(std::string, int, json)));
	//connections from AutoPilot --> SockServer
	QObject::connect(&ap, SIGNAL(sigSocketSendData(std::string, int, json)),
			sock, SLOT(socketSendData(std::string, int, json)));

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

	//connections from PIDParametersDialog --> DubinsScheduler
	QObject::connect(pidParams, SIGNAL(sigStartPathTracking(bool)),
			&dubSched, SLOT(takeMeDown(bool)));

	//connections from DubinsScheduler --> PIDParametersDialog
	QObject::connect(&dubSched, SIGNAL(sigDisplayFlightPhase(QString, QString)),
				pidParams, SLOT(displayFlightPhase(QString, QString)));
	QObject::connect(&dubSched, SIGNAL(sigPathTrackingStatus(bool)),
				pidParams, SLOT(displayPathTrackingStatus(bool)));

	//connections from PIDParametersDialog --> ControlAutomation
	QObject::connect(pidParams, SIGNAL(sigLifeSaverHeightChanged(QString)),
			&ctrlAuto, SLOT(setLifeSaverHeightChanged(QString)));
	QObject::connect(pidParams, SIGNAL(sigApproachStartingAltitudeChanged(QString)),
			&ctrlAuto, SLOT(setApproachStartingAltitudeChanged(QString)));
	//connections from ControlAutomation --> PIDParametersDialog
	QObject::connect(&ctrlAuto, SIGNAL(sigLifeSaverTriggered(void)),
			pidParams, SLOT(clickSetHeight(void)));
	QObject::connect(&ctrlAuto, SIGNAL(sigApproachStartingTriggered(void)),
			pidParams, SLOT(clickTakeMeDown(void)));



	return app.exec();
#endif
}

