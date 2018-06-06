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

#include "json.hpp"
// for convenience
using json = nlohmann::json;

#include <qmetatype.h>

int main(int argc, char *argv[]) {

	QApplication app(argc, argv);

	qRegisterMetaType<json>("json");

	PIDParametersDialog *pidParams;
	pidParams = new PIDParametersDialog;
	pidParams->setWindowTitle(QObject::tr("PID Parameter Selector"));

	pidParams->show();

	DataCenter *dc=DataCenter::getInstance();

	AutoPilot ap;

	SockServer *sock = SockServer::getInstance();
	sock->setDebug(true);

	RequestDispatcher rqd;


	//connections from PIDParametersDialog --> AutoPilot
	QObject::connect(pidParams,
			SIGNAL(sigPidParametersChanged(ctrlType, double, double, double)), &ap,
			SLOT(setControllerParameters(ctrlType, double, double,double)));
	QObject::connect(pidParams, SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)),
			&ap, SLOT(invokeController(ctrlType, bool)));
	QObject::connect(pidParams, SIGNAL(sigRequestedSetValueChanged(ctrlType, double)), &ap,
			SLOT(requestCtrlTargetValue(ctrlType, double)));

	//connections from PIDParametersDialog --> DataCenter
	QObject::connect(pidParams,
			SIGNAL(sigLoggingActiveStateChanged(bool, QFile *)), dc,
			SLOT(invokeLogging(bool, QFile *)));
	QObject::connect(pidParams, SIGNAL(sigSendXPDataRef(const char*, double)), dc,
			SLOT(SendXPDataRef(const char*, double)));

	//connections from AutoPilot --> DataCenter
	QObject::connect(&ap, SIGNAL(sigSendXPDataRef(const char*, double)), dc,
				SLOT(SendXPDataRef(const char*, double)));
	QObject::connect(&ap, SIGNAL(sigSendXPDataRef(const char*, bool)), dc,
				SLOT(SendXPDataRef(const char*, bool)));

	//connections from DataCenter --> PIDParametersDialog
	QObject::connect(dc, SIGNAL(XPlaneConnectionChanged(bool)), pidParams,
			SLOT(setXPlaneConnection(bool)));

	//connections from AutoPilot --> PIDParametersDialog
	QObject::connect(&ap, SIGNAL(sigAttachControllerCurve(ctrlType, QwtPlotCurve*)), pidParams,
			SLOT(attachControllerCurve(ctrlType, QwtPlotCurve*)));
	QObject::connect(&ap, SIGNAL(sigReplotControllerCurve(ctrlType)), pidParams,
			SLOT(replotControllerCurve(ctrlType)));

	//connections from SockServer --> RequestDispatcher
	QObject::connect(sock, SIGNAL(sigDispatchSockMessage(json)), &rqd,
			SLOT(dispatchSockMessage(json)));

	//connections from RequestDispatcher --> DataCenter
	QObject::connect(&rqd, SIGNAL(requested_getPosition()), dc,
			SLOT(on_rqd_requested_getPosition()));
	QObject::connect(&rqd, SIGNAL(requested_getPositionXY()), dc,
			SLOT(on_rpd_requested_getPositionXY()));
	QObject::connect(&rqd, SIGNAL(requested_getOrigin()), dc,
			SLOT(on_rqd_requested_getOrigin()));
	QObject::connect(&rqd, SIGNAL(requested_getPlaneState()), dc,
			SLOT(on_rqd_requested_getPlaneState()));

	//connections from DataCenter --> SockServer
	QObject::connect(dc, SIGNAL(sigSocketSendData(std::string, json)), sock,
			SLOT(socketSendData(std::string, json)));

	return app.exec();
}

