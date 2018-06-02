/*
 * main.cpp
 *
 *  Created on: 25.04.2018
 *      Author: eckstein
 */
#include <QtGui>
#include <qobject.h>
#include <iostream>

#include "pid_parameters.h"
#include "Datacenter.h"
#include "SockServer.h"
#include "RequestDispatcher.h"

#include "json.hpp"
// for convenience
using json = nlohmann::json;

#include <qmetatype.h>

int main(int argc, char *argv[]) {

	QApplication app(argc, argv);

	qRegisterMetaType<json>("json");

//    QTempomat tempomat;
	PIDParametersDialog *pidParams;
	pidParams = new PIDParametersDialog;
	pidParams->setWindowTitle(QObject::tr("PID Parameter Selector"));

	pidParams->show();

	Datacenter dc;

	QObject::connect(pidParams, SIGNAL(sigTempomatActiveStateChanged(bool)),
			&dc, SLOT(invokeTempomat(bool)));
	QObject::connect(pidParams, SIGNAL(sigRequestedSpeedChanged(double)), &dc,
			SLOT(requestTargetSpeed(double)));
	QObject::connect(pidParams,
			SIGNAL(sigPidParametersChanged(double, double, double)), &dc,
			SLOT(setControllerParameters(double, double,double)));

	QObject::connect(pidParams, SIGNAL(sigBankControlActiveStateChanged(bool)),
			&dc, SLOT(invokeBankingControl(bool)));
	QObject::connect(pidParams, SIGNAL(sigRequestedBankingChanged(double)), &dc,
			SLOT(requestTargetBanking(double)));
	QObject::connect(pidParams,
			SIGNAL(sigPidParametersBankChanged(double, double, double)), &dc,
			SLOT(setControllerParametersBanking(double, double,double)));

	QObject::connect(pidParams,
			SIGNAL(sigLoggingActiveStateChanged(bool, QFile *)), &dc,
			SLOT(invokeLogging(bool, QFile *)));

	QObject::connect(pidParams, SIGNAL(sigSetRequestedAltitude(double)), &dc,
			SLOT(setRequestedAltitude(double)));

	QObject::connect(&dc, SIGNAL(XPlaneConnectionChanged(bool)), pidParams,
			SLOT(setXPlaneConnection(bool)));
	QObject::connect(&dc, SIGNAL(attachSpeedCurve(QwtPlotCurve*)), pidParams,
			SLOT(attachSpeedCurve(QwtPlotCurve*)));
	QObject::connect(&dc, SIGNAL(replotSpeedCurve()), pidParams,
			SLOT(replotSpeedCurve()));
	QObject::connect(&dc, SIGNAL(attachRollCurve(QwtPlotCurve*)), pidParams,
			SLOT(attachRollCurve(QwtPlotCurve*)));
	QObject::connect(&dc, SIGNAL(replotRollCurve()), pidParams,
			SLOT(replotRollCurve()));

	SockServer *sock = SockServer::getInstance();
	sock->setDebug(true);
	RequestDispatcher rqd;

	QObject::connect(sock, SIGNAL(sigDispatchSockMessage(json)), &rqd,
			SLOT(dispatchSockMessage(json)));

	QObject::connect(&rqd, SIGNAL(requested_getPosition()), &dc,
			SLOT(on_qpd_requested_getPosition()));
	QObject::connect(&rqd, SIGNAL(requested_getPositionXY()), &dc,
			SLOT(on_qpd_requested_requested_getPositionXY()));
	QObject::connect(&rqd, SIGNAL(requested_getOrigin()), &dc,
			SLOT(on_qpd_requested_getOrigin()));
	QObject::connect(&rqd, SIGNAL(requested_getPlaneState()), &dc,
			SLOT(on_qpd_requested_getPlaneState()));

	QObject::connect(&dc, SIGNAL(sigSocketSendData(std::string, json)), sock,
			SLOT(socketSendData(std::string, json)));

	return app.exec();
}

