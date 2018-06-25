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
class QDebug;
class XPlaneUDPClient;
#include "XPlaneBeaconListener.h"
class PIDParametersDialog;
class QwtPlotCurve;
class QFile;
class QTextStream;

class DataCenter: public QObject {
Q_OBJECT

private:
	DataCenter(QObject* parent = 0);
	DataCenter(DataCenter const &) :
			QObject(0) {
	} // private so it cannot be called

	DataCenter & operator=(DataCenter const &) {
		abort();
	}

public:
	virtual ~DataCenter();

	static DataCenter * getInstance() {
		if (!instance) {
			instance = new DataCenter();
		}
		return instance;
	}

	void setDebug(int _debug) {
		debug = _debug;
	}

	void setClimbControlActive(bool active) {
		curDat.climbControlActive = active;
	}
	void setRollControlActive(bool active) {
		curDat.climbControlActive = active;
	}
	double getVh_ind() {
		return curDat.vh_ind;
	}
	double getVpath() {
		return curDat.vpath;
	}
	double getTrue_theta() {
		return curDat.true_theta;
	}

	double getTrue_phi() {
		return curDat.true_phi;
	}

	double getTrue_airspeed() {
		return curDat.true_airspeed;
	}

public slots:
//void setRequestedAltitude(double altitudeAboveGround);

	void on_rqd_requested_getPosition(int requestId);
	void on_rpd_requested_getPositionXY(int requestId);
	void on_rqd_requested_getOrigin(int requestId);
	void on_rqd_requested_getPlaneState(int requestId);

	void setOrigin(void);
	void invokeLogging(bool active, QFile* fileLog);
	void setElfLocation(double forward, double right, double height, double rotation);

	void SendXPDataRef(const char* dataRefName, double value);
	void SendXPDataRef(const char* dataRefName, bool state);

private slots:
	void timerExpired(void);
	void setConnected(bool connected);

signals:
	void XPlaneConnectionChanged(bool connected);

	void sigSocketSendData(std::string msgType, int requestId, json data);
	void originSetTo(Position_WGS84 origin);
	void sigElfCoordsSet(Position_WGS84 elf, Position_Cartesian elfCart, double elfHeading);


private:
	bool debug = false;

	//Data Items
	Dataset curDat;
	std::vector<Dataset> dataSeries;
	unsigned int count = 0;
	QTimer *basicTimer = NULL;
	const int timerMilliseconds = 100;

	// data arrays for the plot
	static const int plotDataSize = 500;
	double climbRateData[plotDataSize];
	double elevatorData[plotDataSize];
	double rollRateData[plotDataSize];
	double aileronData[plotDataSize];
	double timeData[plotDataSize];
//	QwtPlotCurve *climbCurve=NULL, *elevatorCurve=NULL;
//	QwtPlotCurve *rollCurve=NULL, *aileronCurve=NULL;

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

	bool loggingActive = false;
	QTextStream *outLog = NULL;

	bool originSet = false;

protected:
	static DataCenter *instance;

};

#endif /* SRC_DATACENTER_H_ */
