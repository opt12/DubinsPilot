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

#include <string>
#include <vector>
#include <functional>

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
		const char* dataRef="";
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

signals:
	void sigSendXPDataRef(const char*, bool);
	void sigSendXPDataRef(const char*, double);

	void sigAttachControllerCurve(ctrlType ctrl, QwtPlotCurve* ControllerCurve);

	void sigReplotControllerCurve(ctrlType ctrl);

private:
	static DataCenter *dc;
	QTimer *basicTimer;
	const int timerMilliseconds = 100;
	std::vector<Controller> ctrl;

	int count=0;
	bool debug = true;

private slots:
	void timerExpired(void);
	void updateAllPlots(void);	//adds a new point to the QwtPlotCurves
};

#endif /* AUTO_PILOT_H_ */
