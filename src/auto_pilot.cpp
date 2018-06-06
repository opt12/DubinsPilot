/*
 * auto_pilot.cpp
 *
 *  Created on: 02.06.2018
 *      Author: eckstein
 */

#include <auto_pilot.h>

#include <QObject>
#include <QTimer>
#include <qwt_plot.h>	//to have the QwtPlot::yleft, ... specifiers available
#include <qwt_plot_curve.h>


#include <iostream>

DataCenter *AutoPilot::dc = DataCenter::getInstance();

AutoPilot::AutoPilot(QObject* parent) :
		QObject(parent) {


	basicTimer = new QTimer();
	QObject::connect(basicTimer, SIGNAL(timeout()), this, SLOT(timerExpired()));

	ctrl = std::vector<Controller>(ctrlType::_size());
	ctrl[ctrlType::CLIMB_CONTROL].controller = new PIDControl(0.0, 0.0, 0.0, 0.1, -100.0, 100.0, MANUAL,
			DIRECT);
	ctrl[ctrlType::CLIMB_CONTROL].dataRef = "sim/joystick/artstab_pitch_ratio";

	ctrl[ctrlType::ROLL_CONTROL].controller = new PIDControl(0.0, 0.0, 0.0, 0.1, -50.0, 50.0, MANUAL,
			DIRECT);
	ctrl[ctrlType::ROLL_CONTROL].dataRef = "sim/joystick/artstab_roll_ratio";


	//hier kann ich noch keinen attach ausführen, weil die Verbindung in main noch nicht besteht.
}

AutoPilot::~AutoPilot() {
	// TODO Auto-generated destructor stub
	free(ctrl[ctrlType::CLIMB_CONTROL].controller);
	free(ctrl[ctrlType::ROLL_CONTROL].controller);
	basicTimer->stop();
	free(basicTimer);
}

AutoPilot::Controller::Controller(){
	//initialize the plot data to all zeroes;
	for (int i = 0; i < plotDataSize; ++i) {
		currentValueHistory[i] = 0.0;
		outputValueHistory[i] = 0.0;
		timeData[i] = i;
	}

	currentValueCurve = new QwtPlotCurve;
	currentValueCurve->setSamples(timeData, currentValueHistory, plotDataSize);
	currentValueCurve->setPen(QPen(Qt::green, 2));
	currentValueCurve->setAxes(QwtPlot::xBottom, QwtPlot::yLeft);

	outputValueCurve = new QwtPlotCurve;
	outputValueCurve->setSamples(timeData, outputValueHistory, plotDataSize);
	outputValueCurve->setPen(QPen(Qt::blue, 2));
	outputValueCurve->setAxes(QwtPlot::xBottom, QwtPlot::yRight);
	std::cout<<"\t\tConstructor for inner Controller\n";
}

AutoPilot::Controller::~Controller() {
	free(currentValueCurve);
	free(outputValueCurve);
}

//public Slots:
void AutoPilot::invokeController(ctrlType _ct, bool active) {
	if(!ctrl[_ct].controlActive && active){
		//Das ist nicht die schönste Stelle hier, aber es sollte gehen.
		emit sigAttachControllerCurve(_ct, ctrl[_ct].currentValueCurve);
		emit sigAttachControllerCurve(_ct, ctrl[_ct].outputValueCurve);
		emit sigReplotControllerCurve(_ct);
	}

	if(!basicTimer->isActive()){
		basicTimer->start(timerMilliseconds);
	}
	ctrl[_ct].controlActive = active;
	if (active) {
		std::cout << _ct._to_string() << " invoked\n";
		ctrl[_ct].controller->PIDTuningsSet(ctrl[_ct].p, ctrl[_ct].i, ctrl[_ct].d);
		ctrl[_ct].controller->PIDModeSet(AUTOMATIC);

		//in the if we need the +Operator as seen on https://github.com/aantron/better-enums/issues/23
		if(_ct == +ctrlType::CLIMB_CONTROL || _ct == +ctrlType::ROLL_CONTROL){
			//we need this override to be active for those controllers
			emit sigSendXPDataRef("sim/operation/override/override_artstab", true);
		}
	} else {
		std::cout << _ct._to_string() << " deactivated\n";
		ctrl[_ct].controller->PIDModeSet(MANUAL);
		emit sigSendXPDataRef(ctrl[_ct].dataRef, ctrl[_ct].output);
		if (!ctrl[ctrlType::CLIMB_CONTROL].controlActive && !ctrl[ctrlType::ROLL_CONTROL].controlActive) {
			emit sigSendXPDataRef("sim/operation/override/override_artstab", false);
		}
	}
}

void AutoPilot::requestCtrlTargetValue(ctrlType _ct, double _targetValue) {
	ctrl[_ct].requestedTargetValue = _targetValue;
	if (debug) {
		std::cout << "New targetValue requested for " << _ct._to_string()
				<< ": targetValue= " << _targetValue << ";\n";
	}
}

void AutoPilot::setControllerParameters(ctrlType _ct, double _p, double _i,
		double _d) {
	ctrl[_ct].p = _p;
	ctrl[_ct].i = _i;
	ctrl[_ct].d = _d;
	ctrl[_ct].controller->PIDTuningsSet(_p, _i, _d);
	std::cout << "New controller parameters for " << _ct._to_string() << ": P= "
			<< _p << "; I= " << _i << "; D= " << _d << ";\n";
}


void AutoPilot::timerExpired(void) {
	//set the latest measurements
//	ctrl[ctrlType::CLIMB_CONTROL].currentValue= dc->getVh_ind();
	//TODO
	ctrl[ctrlType::CLIMB_CONTROL].currentValue= dc->getVh_ind();
	ctrl[ctrlType::ROLL_CONTROL].currentValue= dc->getTrue_phi();

	//calculate new control output for all controllers
	for (ctrlType c : ctrlType::_values()){
//		ctrl[c].currentValue = ctrl[c].getMeasurement();
		ctrl[c].controller->PIDSetpointSet(ctrl[c].requestedTargetValue);
		ctrl[c].controller->PIDInputSet(ctrl[c].currentValue);
		ctrl[c].controller->PIDCompute();
		if (ctrl[c].controlActive) {
			ctrl[c].output = ctrl[c].controller->PIDOutputGet();
			emit sigSendXPDataRef(ctrl[c].dataRef, ctrl[c].output);
			if (debug && !(count % 10)) {
				std::cout << c._to_string()<<": currentValue = " << ctrl[c].currentValue
						<< ";\requestedTargetValue = " << ctrl[c].requestedTargetValue
						<< ";\tDeviation= " << ctrl[c].requestedTargetValue - ctrl[c].currentValue
						<< "; \n";
				std::cout << "\tNew output value is " << ctrl[c].output << "\n";
			}
		}
	}
	QTimer::singleShot(0, this, SLOT(updateAllPlots()));
	++count;
}

void AutoPilot::updateAllPlots() {
	for (ctrlType c : ctrlType::_values()){
		ctrl[c].updatePlot();	//updatePlot() is private and hence cannot be called from outside;
		emit sigReplotControllerCurve(c);
	}
}

void AutoPilot::Controller::updatePlot(void) {
	// add the latest data to the plot
	//TODO check if this is really efficient, but works for the moment
	memmove(currentValueHistory, currentValueHistory + 1,
			(plotDataSize - 1) * sizeof(double));
	memmove(outputValueHistory, outputValueHistory + 1,
			(plotDataSize - 1) * sizeof(double));

	currentValueHistory[plotDataSize - 1] = currentValue;
	outputValueHistory[plotDataSize - 1] = output;

	currentValueCurve->setSamples(timeData, currentValueHistory, plotDataSize);
	outputValueCurve->setSamples(timeData, outputValueHistory, plotDataSize);
}


