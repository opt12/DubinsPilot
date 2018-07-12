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
#include "utils.h"

#include <iostream>

DataCenter *AutoPilot::dc = DataCenter::getInstance();

AutoPilot::AutoPilot(QObject* parent) :
		QObject(parent) {

	basicTimer = new QTimer();
	QObject::connect(basicTimer, SIGNAL(timeout()), this, SLOT(timerExpired()));

	ctrl = std::vector<Controller>(ctrlType::_size());
	ctrl[ctrlType::CLIMB_CONTROL].controller = new PIDControl(0.0, 0.0, 0.0,
			0.1, -1.0, 1.0, MANUAL, DIRECT);
	ctrl[ctrlType::CLIMB_CONTROL].publishOutput = [this](double _output)
	{
		emit sigSendXPDataRef("sim/joystick/yoke_pitch_ratio", _output);
	};
	ctrl[ctrlType::CLIMB_CONTROL].getMeasurement = [this](void)
	{ // wir müssen den Gleitwinkel selber rechnen aus true_airspeed und sinkrate
	  // der von XPlane ausgegebene Winkel ist bezogen auf ground_speed und damit bei Wind unbrauchbar.
				double tas = dc->getTrue_airspeed();
				double sinkrate = dc->getVh_ind();
	  //in case we have no airspeed measurement yet, we must avoid div by zero and just return the requested target value
	  // dirty but effective.
				return tas != 0.0? to_degrees(atan(sinkrate/tas)) : ctrl[ctrlType::CLIMB_CONTROL].requestedTargetValue;
			};

	ctrl[ctrlType::ROLL_CONTROL].controller = new PIDControl(0.0, 0.0, 0.0, 0.1,
			-1.0, 1.0, MANUAL, DIRECT);
	ctrl[ctrlType::ROLL_CONTROL].publishOutput = [this](double _output)
	{
		emit sigSendXPDataRef("sim/joystick/yoke_roll_ratio", _output);
	};
	ctrl[ctrlType::ROLL_CONTROL].getMeasurement =
			[this](void) {return dc->getTrue_phi();};

	ctrl[ctrlType::HEADING_CONTROL].controller = new PIDControl(1.0, 0.0, 0.0,
			0.1, -20.0, 20.0, MANUAL, REVERSE);
	ctrl[ctrlType::HEADING_CONTROL].publishOutput = [this](double output)
	{
		ctrl[ctrlType::ROLL_CONTROL].requestedTargetValue = output;	//set the roll target for the lower level controller
			emit sigRequestRoll(output);//adapt the knob in the GUI
		};
	ctrl[ctrlType::HEADING_CONTROL].getMeasurement = [this](void) {
		double true_psi = dc->getTrue_psi();
		return true_psi;
	};
	ctrl[ctrlType::HEADING_CONTROL].controller->calculateError =
			[this](double _setpoint, double _input)
			{
				return getAngularDifference(_setpoint, _input);
			};

	ctrl[ctrlType::RADIUS_CONTROL].controller = new PIDControl(0.0, 0.0, 0.0,
			0.1, -10.0, 10.0, MANUAL, DIRECT);
	ctrl[ctrlType::RADIUS_CONTROL].getMeasurement = [this](void) {
		// the target value is the radius of the circle
			Position curPos = dc->getPosition();
			Position displacedCircleCenter = circleCenter+(dc->getWindDisplacement()-initialDisplacement);
			return curPos.getDistanceCart(displacedCircleCenter);
		};
	ctrl[ctrlType::RADIUS_CONTROL].publishOutput = [this](double output)
	{
		// TODO erstmal nix machen, sondern nur staunen
		//TODO XXX Hier können wir das roll limit noch nachkorrigieren mit Regler
		circleRadiusCorrection = output;
//			ctrl[ctrlType::ROLL_CONTROL].requestedTargetValue += output;//add the radius correction
//			emit sigRequestRoll(ctrl[ctrlType::ROLL_CONTROL].requestedTargetValue);//adapt the knob in the GUI
		};

	//hier kann ich noch keinen attach ausführen, weil die Verbindung in main noch nicht besteht.
	// Aber ich kann den attach in die event queue einreihen und dann geht das.
	QTimer::singleShot(0, this, SLOT(attachPlots(void)));

	//Den Timer können wir gleich starten, dann laufen die Plots auch schon los.
	basicTimer->start(timerMilliseconds);

}

AutoPilot::~AutoPilot() {
	free(ctrl[ctrlType::CLIMB_CONTROL].controller);
	free(ctrl[ctrlType::ROLL_CONTROL].controller);
	free(ctrl[ctrlType::HEADING_CONTROL].controller);
	free(ctrl[ctrlType::RADIUS_CONTROL].controller);
	basicTimer->stop();
	free(basicTimer);
}

AutoPilot::Controller::Controller() {
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
	std::cout << "\t\tConstructor for inner Controller\n";
}

AutoPilot::Controller::~Controller() {
	free(currentValueCurve);
	free(outputValueCurve);
}

//public Slots:
void AutoPilot::invokeController(ctrlType _ct, bool active) {
	if (!basicTimer->isActive()) {
		basicTimer->start(timerMilliseconds);
	}
	ctrl[_ct].controlActive = active;
	if (active) {
		std::cout << _ct._to_string() << " invoked\n";
		ctrl[_ct].controller->PIDTuningsSet(ctrl[_ct].p, ctrl[_ct].i,
				ctrl[_ct].d);
		ctrl[_ct].controller->PIDModeSet(AUTOMATIC);

		//betterEnums: in the if we need the +Operator as seen on https://github.com/aantron/better-enums/issues/23
		if (_ct == +ctrlType::CLIMB_CONTROL) {
			//we need this override to be active for those controllers
			emit sigSendXPDataRef(
					"sim/operation/override/override_joystick_pitch", true);
		}
		if (_ct == +ctrlType::ROLL_CONTROL) {
			//we need this override to be active for those controllers
			emit sigSendXPDataRef(
					"sim/operation/override/override_joystick_roll", true);
		}
		if (_ct == +ctrlType::RADIUS_CONTROL) {
			circleFlightActive = active;
		}
	} else {	//deactivation
		std::cout << _ct._to_string() << " deactivated\n";
		ctrl[_ct].controller->PIDModeSet(MANUAL);
		ctrl[_ct].publishOutput(ctrl[_ct].output);
//		emit sigSendXPDataRef(ctrl[_ct].dataRef, ctrl[_ct].output);
		if (!ctrl[ctrlType::CLIMB_CONTROL].controlActive) {
			emit sigSendXPDataRef(
					"sim/operation/override/override_joystick_pitch", false);
		}
		if (!ctrl[ctrlType::ROLL_CONTROL].controlActive) {
			emit sigSendXPDataRef(
					"sim/operation/override/override_joystick_roll", false);
		}
		if (_ct == +ctrlType::RADIUS_CONTROL) {
			// deactivate the circle flight
			circleFlightActive = active;
			// hide the circle in the visualization
			emit sigSocketSendData(std::string("CIRCLE_DATA"), 0, getCircleDataAsJson());
			// set the heading to the current value to leave the circle in a tangential way
			Position displacedCircleCenter = circleCenter+(dc->getWindDisplacement()-initialDisplacement);
			double circleAngle = displacedCircleCenter.getHeadingCart(dc->getPosition());
			double newHeading = circleAngle + (circleDirectionLeft?-90.0:+90.0);// + circleRadiusCorrection;
			ctrl[ctrlType::HEADING_CONTROL].requestedTargetValue = newHeading;
			emit sigRequestHeading(newHeading); //adapt the knob in the GUI

		}
		if (_ct == +ctrlType::HEADING_CONTROL) {
			//we want to go straight afterwards
			ctrl[ctrlType::ROLL_CONTROL].requestedTargetValue = 0.0;//set the roll target for the lower level controller
			emit sigRequestRoll(0.0);	//adapt the knob in the GUI
		}
	}
}

void AutoPilot::requestCtrlTargetValue(ctrlType _ct, double _targetValue) {
	ctrl[_ct].requestedTargetValue = _targetValue;
	if (false && debug) {
		std::cout << "New targetValue requested for " << _ct._to_string()
				<< ": targetValue= " << _targetValue << ";\n";
		dc->setRequestedTargetValue(_ct, _targetValue);
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

void AutoPilot::requestCircleDirection(bool isLeftCircle, double radius) {
	circleDirectionLeft = isLeftCircle;
	//set the controller direction
	isLeftCircle ?
			ctrl[ctrlType::RADIUS_CONTROL].controller->PIDControllerDirectionSet(
					DIRECT) :
			ctrl[ctrlType::RADIUS_CONTROL].controller->PIDControllerDirectionSet(
					REVERSE);
	//set the centerpoint of the circle. This is either left or right of the current position;
	double deltaX, deltaY;
	rotatePoint(0, radius, dc->getTrue_psi() + (isLeftCircle ? -90.0 : +90),
			deltaX, deltaY);
	circleCenter = Position(dc->getPosition().getPosition_WGS84(), deltaX,
			deltaY, 0.0);
	initialDisplacement = dc->getWindDisplacement();
	//TODO I Anteil im Regler auf jeden Fall zurücksetzen.
	ctrl[ctrlType::RADIUS_CONTROL].controller->PIDResetInternals();
	std::cout << "Current_Position: \t"<< dc->getPosition().getPosition_Cart().asJson()<< std::endl;
	std::cout << "Circle Center Po: \t"<< circleCenter.getPosition_Cart().asJson()<< std::endl;
}

json AutoPilot::getCircleDataAsJson(void){
	json j;
	Position displacedCircleCenter = circleCenter+(dc->getWindDisplacement()-initialDisplacement);
	if(circleFlightActive){
		j["center"] = displacedCircleCenter.getPosition_WGS84().asJson();
		j["radius"] = ctrl[ctrlType::RADIUS_CONTROL].requestedTargetValue;
		j["directionLeft"] = circleDirectionLeft;
	}
	else {
		j = json::object({});	//leave the object empty
	}
	return j;
}

void AutoPilot::timerExpired(void) {

	if(circleFlightActive){
		// when in circle flight, we need to permanently update the heading
		Position displacedCircleCenter = circleCenter+(dc->getWindDisplacement()-initialDisplacement);
		double circleAngle = displacedCircleCenter.getHeadingCart(dc->getPosition());
		// during the circle flight we permanently oversteer to hold the plane in max. roll angle
		double newHeading = circleAngle + (circleDirectionLeft?-90.0-45:+90.0+45);
		// this roll angle is calculated with respect to the requested radius
		double bankingLimit = getRollAngle(dc->getTrue_airspeed(), ctrl[ctrlType::RADIUS_CONTROL].requestedTargetValue);
		ctrl[ctrlType::HEADING_CONTROL].controller->PIDOutputLimitsSet(
				-bankingLimit + circleRadiusCorrection,
				bankingLimit + circleRadiusCorrection);
		std::cout << "Banking Limit set to " << bankingLimit << " [deg] + "
				<< circleRadiusCorrection << " [deg]\n";
		ctrl[ctrlType::HEADING_CONTROL].requestedTargetValue = fmod(newHeading, 360.0);
//		std::cout << "CircleFlight Direction = "<< ctrl[ctrlType::HEADING_CONTROL].requestedTargetValue<<"\n";
		emit sigRequestHeading(newHeading);//adapt the knob in the GUI
		emit sigSocketSendData(std::string("CIRCLE_DATA"), 0, getCircleDataAsJson());
	}

	//calculate new control output for all controllers
	for (ctrlType c : ctrlType::_values()) {
		ctrl[c].currentValue = ctrl[c].getMeasurement();
		ctrl[c].controller->PIDSetpointSet(ctrl[c].requestedTargetValue);
		ctrl[c].controller->PIDInputSet(ctrl[c].currentValue);
		ctrl[c].controller->PIDCompute();
		if (ctrl[c].controlActive) {
			ctrl[c].output = ctrl[c].controller->PIDOutputGet();
			ctrl[c].publishOutput(ctrl[c].output);
			//TODO false wieder rausnehmen und dafür debug richtig setzen in dialogbox
			if (false && debug && !(count % 10)) {
				std::cout << c._to_string() << ": currentValue = "
						<< ctrl[c].currentValue << ";\trequestedTargetValue = "
						<< ctrl[c].requestedTargetValue << ";\tDeviation= "
						<< ctrl[c].controller->lastErrorGet() << "; \n";
				std::cout << "\tNew output value is " << ctrl[c].output << "\n";
			}
			dc->setControllerOutputs(c, ctrl[c].output);
		}
	}
	QTimer::singleShot(0, this, SLOT(updateAllPlots()));
	++count;
}

void AutoPilot::updateAllPlots() {
	for (ctrlType c : ctrlType::_values()) {
		ctrl[c].updatePlot();//updatePlot() is private and hence cannot be called from outside;
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

void AutoPilot::attachPlots(void) {
	std::cout << "attaching the plots now \n";
	for (ctrlType _ct : ctrlType::_values()) {
		emit sigAttachControllerCurve(_ct, ctrl[_ct].currentValueCurve);
		emit sigAttachControllerCurve(_ct, ctrl[_ct].outputValueCurve);
		emit sigReplotControllerCurve(_ct);
	}
}

