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
#include "SegmentStatistics.h"

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

	void setControllerOutputs(ctrlType _c, double _outputValue) {
		curDat.controllerOutputs[_c] = _outputValue;
	}
	void setRequestedTargetValue(ctrlType _c, double _requestedValue) {
		curDat.requestedTargetVals[_c] = _requestedValue;
	}
	void setCurrentValues(ctrlType _c, double _requestedValue) {
		curDat.currentValue[_c] = _requestedValue;
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

	double getTrue_psi() {
		return curDat.true_psi;
	}

	// TODO wieder rausnehmen, ist nur zum Testen
	void setTrue_psi(double _true_psi) {
		curDat.true_psi = _true_psi;
		emit sigSocketSendData(std::string("PLANE_STATE"), 0,
				curDat.asJson());
	}

	double getTrue_airspeed() {
		return curDat.true_airspeed;
	}

	Position_WGS84 getPosition_WGS84() {
		return curDat.pos.getOrigin_WGS84();
	}

	Position getPosition() {
		return curDat.pos;
	}

	Position_Cartesian getWindDisplacement() {
		return curDat.windDisplacement;
	}

	DubinsPath* getCurrentDubinsPath(){
		return &curDat.db;
	}

	bool isSimulationPaused() const{
		return isPaused;
	}

	const SegmentStatistics* getSegmentStats() const {
		return &segmentStats;
	}

public slots:

//TODO Prüfe, ob das alles nicht einfach rauskann, weil ich es aktuell nicht benutze
//	void on_rqd_requested_getPosition(int requestId);
//	void on_rpd_requested_getPositionXY(int requestId);
//	void on_rqd_requested_getOrigin(int requestId);
//	void on_rqd_requested_getPlaneState(int requestId);

	void setOrigin(void);
	void setIsSimulationPaused(bool _isPaused) {
		isPaused = _isPaused;
	}
	void resetWindDisplacement(void);
	void invokeLogging(bool active, QFile* fileLog);
	void setElfLocation(double forward, double right, double height, double rotation, pathTypeEnum pathType);
	void setElfLocation(Position_WGS84 elfPosition, double elfHeading, pathTypeEnum pathType);
	void resetElfLocation(void);

	void SendXPDataRef(const char* dataRefName, double value);
	void SendXPDataRef(const char* dataRefName, bool state);

	void changeSegmentStatisticsState(bool isActive);
	void updateSegmentStatistics(void);

	void setFlightPathCharacteristics(double valClimbRateStraight,
			double valClimbRateCircle, double valAutoCircleRadius);

	double getAutoCircleRadius() const {
		return autoCircleRadius;
	}

	double getAutoGlideAngleCircle() const {
		return autoGlideAngleCircle;
	}

	double getAutoGlideAngleStraight() const {
		return autoGlideAngleStraight;
	}

	double getAutoGlideRatioCircle() const {
		return autoGlideRatioCircle;
	}

	double getAutoGlideRatioStraight() const {
		return autoGlideRatioStraight;
	}

private slots:
	void timerExpired(void);
	void setConnected(bool connected);
	void calculateDubinsPath(pathTypeEnum pathType);


signals:
	void XPlaneConnectionChanged(bool connected);

	void sigSocketSendData(std::string msgType, int requestId, json data);
	void originSetTo(Position_WGS84 origin);
	void sigElfCoordsSet(Position_WGS84 elf, double elfHeading, bool pathFeasible);
	void sigWindChanged(double windDirFrom, double windVelocity);
	void sigCalculateDubinsPath(pathTypeEnum pathType);

private:
	bool debug = false;

	static int UPDATE_RATE;

	//Data Items
	Dataset curDat;
	SegmentStatistics segmentStats = SegmentStatistics();

	double autoGlideAngleStraight=0.0, autoGlideRatioStraight=0.0;
	double autoGlideAngleCircle=0.0, autoGlideRatioCircle=0.0;
	double autoCircleRadius=0.0;

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

	bool connected = false;
	bool isPaused = false;

	bool loggingActive = false;
	QTextStream *outLog = NULL;

	bool originSet = false;

	std::string host = "";
	uint16_t port = 0;

protected:
	static DataCenter *instance;

};

#endif /* SRC_DATACENTER_H_ */
