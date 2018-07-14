/*
 * auto_pilot.h
 *
 *  Created on: 02.06.2018
 *      Author: eckstein
 */

#ifndef AUTO_PILOT_H_
#define AUTO_PILOT_H_

#include <DataCenter.h>

#include <QObject>

#include <pid_controller.h>
#include "enumDeclarations.h"
#include "Position.h"

#include <string>
#include <vector>
#include <functional>

#include "json.hpp"
// for convenience
using json = nlohmann::json;


class QTimer;

class AutoPilot: public QObject {
	Q_OBJECT

private:
	class Controller {
	public:
		Controller();
		virtual ~Controller();

	private:
		// data arrays for the plot
		static const int plotDataSize = 500;
		double currentValueHistory[plotDataSize];
		double outputValueHistory[plotDataSize];
		double timeData[plotDataSize];


	public:
		PIDControl *controller=NULL;
		double p=0.0, i=0.0, d=0.0;
		double requestedTargetValue=0.0;
		double currentValue=0.0;
		std::function<double(void)> getMeasurement = [this](void){return 0.0;};
		std::function<void(double output)> publishOutput = [this](double output) {return output;};
		double output=0.0;
		bool controlActive = false;
		QwtPlotCurve *currentValueCurve=NULL, *outputValueCurve=NULL;
		void updatePlot(void);	//adds a new point to the QwtPlotCurves
	};

public:
	AutoPilot(QObject* parent = 0);
	virtual ~AutoPilot();

public slots:
	void invokeController(ctrlType ctrl, bool active);
	void requestCtrlTargetValue(ctrlType ctrl, double target);
	void setControllerParameters(ctrlType ctrl, double P, double I, double D);
	void requestCircleDirection(bool isLeftCircle, double radius);

	private slots:
	void attachPlots(void);

signals:
	void sigSendXPDataRef(const char*, bool);
	void sigSendXPDataRef(const char*, double);

	void sigAttachControllerCurve(ctrlType ctrl, QwtPlotCurve* ControllerCurve);

	void sigReplotControllerCurve(ctrlType ctrl);

	void sigRequestRoll(double requestedRoll);
	void sigRequestClimb(double requestedClimb);
	void sigRequestHeading(double requestedHeading);

	void sigSocketSendData(std::string msgType, int requestId, json data);

private:
	static DataCenter *dc;
	QTimer *basicTimer;
	const int timerMilliseconds = 100;
	std::vector<Controller> ctrl;

	int count=0;
	bool debug = true;

	//For flying circles
	bool circleDirectionLeft = true;	//TODO kann ich das auch sinnvoll im Controller unterbringen?
	Position circleCenter = Position();
	bool circleFlightActive = false;
	double circleRadiusCorrection = 0.0;
	Position_Cartesian initialDisplacement={0.0, 0.0, 0.0};

	json getCircleDataAsJson(void);

	bool dubinsSchedulerActive = false;

private slots:
	void timerExpired(void);
	void updateAllPlots(void);	//adds a new point to the QwtPlotCurves
};

#endif /* AUTO_PILOT_H_ */
