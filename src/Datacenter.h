/*
 * Datacenter.h
 *
 *  Created on: 05.05.2018
 *      Author: eckstein
 */

#ifndef SRC_DATACENTER_H_
#define SRC_DATACENTER_H_

#include <QObject>

#include "Dataset.h"
#include <vector>
#include "json.hpp"
// for convenience
using json = nlohmann::json;


class QTimer;
class XPlaneUDPClient;
#include "XPlaneBeaconListener.h"
class PIDParametersDialog;
class QwtPlot;
class QwtPlotCurve;
class PIDControl;
class QFile;
class QTextStream;

class Datacenter: public QObject {
	Q_OBJECT

public:
	Datacenter(QObject* parent = 0);
	virtual ~Datacenter();


public slots:
void invokeTempomat(bool active);
void requestTargetSpeed(double speed);
void setControllerParameters(double P, double I, double D);
void invokeBankingControl(bool active);
void requestTargetBanking(double speed);
void setControllerParametersBanking(double P, double I, double D);
void setRequestedAltitude(double altitudeAboveGround);

void on_qpd_requested_getPosition();
void on_qpd_requested_requested_getPositionXY();
void on_qpd_requested_getOrigin();
void on_qpd_requested_getPlaneState();

void invokeLogging(bool active, QFile* fileLog);

private slots:
	void timerExpired(void);
	void updatePlot(void);	//adds a new point to the QwtPlotCurves


signals:
	void XPlaneConnectionChanged(bool connected);
	void attachSpeedCurve(QwtPlotCurve* speedCurve);
	void replotSpeedCurve(void);
	//TODO das muss noch zusammengefasstwerden in ein generisches Signal mit Scaling-Hint
//	void attachElevatorCurve(QwtPlotCurve* elevatorCurve);
	void replotElevatorCurve(void);

	void attachRollCurve(QwtPlotCurve* rollCurve);
	void replotRollCurve(void);

	void sigSocketSendData(std::string, json);

private:
	//Data Items
	Dataset curDat;
	std::vector<Dataset> dataSeries;
	unsigned int count=0;
	QTimer *basicTimer;
	const int timerMilliseconds = 100;

	// data arrays for the plot
	static const int plotDataSize = 500;
	double climbRateData[plotDataSize];
	double elevatorData[plotDataSize];
	double rollRateData[plotDataSize];
	double aileronData[plotDataSize];
	double timeData[plotDataSize];
	QwtPlotCurve *climbCurve, *elevatorCurve;
	QwtPlotCurve *rollCurve, *aileronCurve;


	//Tempomat
	//all the relevant data for the tempomat are stored here
	PIDControl *speedController;

	//Banking controller
	PIDControl *bankController;

	//XPlane connection
	XPlaneUDPClient *xp = NULL;
	void receiverBeaconCallback(XPlaneBeaconListener::XPlaneServer server,
			bool exists);
	void receiverCallbackFloat(std::string dataref, float value);
	void receiverCallbackString(std::string dataref, std::string value);
	void connectToXPlane();

	std::string host = "";
	uint16_t port = 0;
	bool connected = false;
	void setConnected(bool connected);

	bool loggingActive = false;
	QTextStream *outLog=NULL;


};

#endif /* SRC_DATACENTER_H_ */
