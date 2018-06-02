/*
 * Datacenter.cpp
 *
 *  Created on: 05.05.2018
 *      Author: eckstein
 */
#include "Datacenter.h"

#include <pid_controller.h>

#include <QObject>

#include "Dataset.h"
#include <vector>
#include <iostream>
#include <functional>
#include <string>

#include <QTimer>
#include "XPlaneBeaconListener.h"
#include "XPlaneUDPClient.h"
#include <qwt_plot.h>	//to have the QwtPlot::yleft, ... specifiers available
#include <qwt_plot_curve.h>
#include <qtextstream.h>
#include <qfile.h>

constexpr
static unsigned int hash(const char* str, int h = 0) { //TODO check if this hashing is a good idea...
													   //https://stackoverflow.com/questions/16388510/evaluate-a-string-with-a-switch-in-c/16388610#16388610
	return !str[h] ? 5381 : (hash(str, h + 1) * 33) ^ str[h];
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Datacenter::Datacenter(QObject* parent) :
		QObject(parent) {

	elevatorCurve = new QwtPlotCurve;
	climbCurve = new QwtPlotCurve;
	rollCurve = new QwtPlotCurve;
	aileronCurve = new QwtPlotCurve;
	{	//TODO initialize Plot data
		//initialize Plot Data to 0

		Dataset ds;
		for (int i = 0; i < plotDataSize; ++i) {
			climbRateData[i] = 0;
			elevatorData[i] = 0;
			rollRateData[i] = 0;
			aileronData[i] = 0;
			timeData[i] = i;
			//TODO check if dumy data in the archive is useful;
			dataSeries.push_back(ds);
			++count;
		}
		climbCurve->setSamples(climbRateData, timeData, plotDataSize);
		climbCurve->setPen(QPen(Qt::green, 2));
		climbCurve->setAxes(QwtPlot::xBottom, QwtPlot::yLeft);

		elevatorCurve = new QwtPlotCurve;
		elevatorCurve->setSamples(elevatorData, timeData, plotDataSize);
		elevatorCurve->setPen(QPen(Qt::blue, 2));
		elevatorCurve->setAxes(QwtPlot::xBottom, QwtPlot::yRight);

		rollCurve->setSamples(rollRateData, timeData, plotDataSize);
		rollCurve->setPen(QPen(Qt::green, 2));
		rollCurve->setAxes(QwtPlot::xBottom, QwtPlot::yLeft);

		aileronCurve = new QwtPlotCurve;
		aileronCurve->setSamples(aileronData, timeData, plotDataSize);
		aileronCurve->setPen(QPen(Qt::blue, 2));
		aileronCurve->setAxes(QwtPlot::xBottom, QwtPlot::yRight);

	}

//	speedController = new PIDControl(0.0, 0.0, 0.0, 0.1,
//			-1.0, 1.0, MANUAL, REVERSE);
	speedController = new PIDControl(0.0, 0.0, 0.0, 0.1, -100.0, 100.0, MANUAL,
			DIRECT);

	bankController = new PIDControl(0.0, 0.0, 0.0, 0.1, -50.0, 50.0, MANUAL,
			DIRECT);

	curDat.climbControlActive = false;
	curDat.rollControlActive = false;

	basicTimer = new QTimer();
	QObject::connect(basicTimer, SIGNAL(timeout()), this, SLOT(timerExpired()));

	//TODO XXX comment out to Debug
	//	connectToXPlane();
	setConnected(true);
}

Datacenter::~Datacenter() {
	// TODO Auto-generated destructor stub
	if (xp) {
		xp->setDataRef("sim/operation/override/override_artstab", false);
		xp->setDataRef("sim/operation/override/override_joystick_pitch", false);
		xp->setDataRef("sim/operation/override/override_joystick_roll", false);
	}
	free(elevatorCurve);
	free(climbCurve);
	free(rollCurve);
	free(aileronCurve);
	basicTimer->stop();
	free(basicTimer);
	free(speedController);
}

//public Slots:
void Datacenter::invokeTempomat(bool active) {
	curDat.climbControlActive = active;
	if (active) {
		std::cout << "Tempomat invoked\n";
		speedController->PIDTuningsSet(curDat.PSpeed, curDat.ISpeed,
				curDat.DSpeed);
		speedController->PIDModeSet(AUTOMATIC);

		if (xp)
			xp->setDataRef("sim/operation/override/override_artstab", true);
//		if(xp)	xp->setDataRef("sim/operation/override/override_joystick_pitch", true);
	} else {
		std::cout << "Tempomat deactivated\n";
		speedController->PIDModeSet(MANUAL);
//		curDat.artstab_pitch_ratio = 0.0;
		xp->setDataRef("sim/joystick/artstab_pitch_ratio",
				curDat.artstab_pitch_ratio);
		if (!curDat.climbControlActive && !curDat.rollControlActive) {
			if (xp)
				xp->setDataRef("sim/operation/override/override_artstab",
						false);

		}
		if (xp)
			xp->setDataRef("sim/operation/override/override_joystick_pitch",
					false);
	}
}

void Datacenter::requestTargetSpeed(double climbRate) {
	curDat.requestedClimbRate = climbRate;
	std::cout << "New speed requested: climbRate= " << climbRate
			<< " ft/min;\n";
}

void Datacenter::setControllerParameters(double P, double I, double D) {
	curDat.PSpeed = P;
	curDat.ISpeed = I;
	curDat.DSpeed = D;
	speedController->PIDTuningsSet(P, I, D);
	std::cout << "New controller Parameters set: P= " << P << "; I= " << I
			<< "; D= " << D << ";\n";
}

void Datacenter::invokeBankingControl(bool active) {
	curDat.rollControlActive = active;
	if (active) {
		std::cout << "Banking control invoked\n";
		bankController->PIDTuningsSet(curDat.PRoll, curDat.IRoll, curDat.DRoll);
		bankController->PIDModeSet(AUTOMATIC);

		if (xp)
			xp->setDataRef("sim/operation/override/override_artstab", true);
	} else {
		std::cout << "Banking control deactivated\n";
		bankController->PIDModeSet(MANUAL);
//		curDat.artstab_roll_ratio = 0.0;
		xp->setDataRef("sim/joystick/artstab_roll_ratio",
				curDat.artstab_roll_ratio);
		if (!curDat.climbControlActive && !curDat.rollControlActive) {
			if (xp)
				xp->setDataRef("sim/operation/override/override_artstab",
						false);

		}
		if (xp)
			xp->setDataRef("sim/operation/override/override_joystick_roll",
					false);
	}
}

void Datacenter::requestTargetBanking(double banking) {
	curDat.requestedRoll = banking;
	std::cout << "New banking requested: banking= " << banking << " °;\n";
}

void Datacenter::setControllerParametersBanking(double P, double I, double D) {
	curDat.PRoll = P;
	curDat.IRoll = I;
	curDat.DRoll = D;
	bankController->PIDTuningsSet(P, I, D);
	std::cout << "New banking controller Parameters set: P= " << P << "; I= "
			<< I << "; D= " << D << ";\n";
}

void Datacenter::setRequestedAltitude(double altitudeAboveGround) {
	if (xp) {
		xp->setDataRef("sim/flightmodel/position/local_y", altitudeAboveGround);
		std::cout << "Altitude set to " << altitudeAboveGround << " meters"
				<< std::endl;
	}
}

void Datacenter::invokeLogging(bool active, QFile* fileLog) {
	loggingActive = active;
	if (active) {
		outLog = new QTextStream(fileLog);
		Position::setOrigin(curDat.getPosition_WGS84());
		QString origin = "origin:\n"
				"latitude;"
				+ QString::number(curDat.getPosition_WGS84().lati, 'g', 12)
				+ ";"
						"longitude;"
				+ QString::number(curDat.getPosition_WGS84().longi, 'g', 12)
				+ ";"
						"height;"
				+ QString::number(curDat.getPosition_WGS84().height, 'g', 6)
				+ ";\n";
		*outLog << origin;
		*outLog << Dataset::csvHeading();

	} else {
		free(outLog);
	}
	std::cout << "loggingActive: " << loggingActive << "; File: "
			<< fileLog->fileName().toStdString() << std::endl;
}

//Private Slots:
void Datacenter::timerExpired(void) {
	//calculate new control output
	speedController->PIDSetpointSet(curDat.requestedClimbRate);
	speedController->PIDInputSet(curDat.vh_ind);
//	speedController->PIDInputSet(curDat.true_theta);	//for pitch control
	speedController->PIDCompute();

	bankController->PIDSetpointSet(curDat.requestedRoll);
	bankController->PIDInputSet(curDat.true_phi);
	bankController->PIDCompute();

	if (loggingActive) {
		*outLog << curDat.csvDataRecord();
//		std::cout << curDat.csvDataRecord().toStdString();
	}

	if (this->curDat.climbControlActive) {
//		curDat.yoke_pitch_ratio += speedController->PIDOutputGet();//*timerMilliseconds/1000;
		double delta_pitch_ratio = speedController->PIDOutputGet()
				- curDat.artstab_pitch_ratio;
		//apply a nonlinear function to the delta_pitch_ratio; Maybe this helps
		delta_pitch_ratio = sgn(delta_pitch_ratio)*(fabs(delta_pitch_ratio));

		curDat.artstab_pitch_ratio += delta_pitch_ratio;
//		curDat.artstab_pitch_ratio = speedController->PIDOutputGet();
		if (!(count % 10)) {
			std::cout << "curDat.vh_ind= " << curDat.vh_ind
					<< ";\tcurDat.requestedClimbRate= "
					<< curDat.requestedClimbRate << ";\tDeviation= "
					<< curDat.requestedClimbRate - curDat.vh_ind << "; \n";
			std::cout << "\tNew yoke_pitch value of "
					<< curDat.artstab_pitch_ratio << "\n";
//			std::cout << "curDat.true_theta= " << curDat.true_theta	//for pitch control
//					<< ";\tcurDat.requestedClimbRate= "
//					<< curDat.requestedClimbRate << ";\tDeviation= "
//					<< curDat.requestedClimbRate - curDat.true_theta << "; \n";
//			std::cout << "\tNew yoke_pitch value of "
//					<< curDat.artstab_pitch_ratio << "\n";
			json data;
			data["happy"]= true;
			data["pi"] = 3.141;
			emit sigSocketSendData(std::string("message"), data);
		}
		if (NULL != xp) {	//do this not before we properly created an object
			xp->setDataRef("sim/joystick/artstab_pitch_ratio",
					curDat.artstab_pitch_ratio
							+ speedController->PIDOutputGet());

			//https://developer.x-plane.com/2013/05/using-the-right-wing-datarefs/
		}
	} else {
		curDat.artstab_pitch_ratio = 0.0;
	}

	if (this->curDat.rollControlActive) {
		curDat.artstab_roll_ratio = bankController->PIDOutputGet();
		if (!(count % 10)) {
			std::cout << "curDat.phi= " << curDat.true_phi
					<< ";\tcurDat.requestedRoll= " << curDat.requestedRoll
					<< ";\tDeviation= "
					<< curDat.requestedRoll - curDat.true_phi << "; \n";
			std::cout << "\tNew yoke_roll value of "
					<< curDat.artstab_roll_ratio << "\n";
		}
		if (NULL != xp) {	//do this not before we properly created an object
//			xp->setDataRef("sim/joystick/yoke_roll_ratio",
//					curDat.yoke_roll_ratio);
			xp->setDataRef("sim/joystick/artstab_roll_ratio",
					curDat.artstab_roll_ratio);

			//https://developer.x-plane.com/2013/05/using-the-right-wing-datarefs/
//			xp->setDataRef("sim/flightmodel2/wing/elevator1_deg[8]",
//					curDat.yoke_pitch_ratio);
//			xp->setDataRef("sim/flightmodel2/wing/elevator1_deg[9]",
//					curDat.yoke_pitch_ratio);
//			xp->setDataRef("sim/flightmodel2/wing/elevator2_deg[8]",
//					curDat.yoke_pitch_ratio);
//			xp->setDataRef("sim/flightmodel2/wing/elevator2_deg[9]",
//					curDat.yoke_pitch_ratio);
		}
	} else {
		curDat.artstab_roll_ratio = 0.0;
	}

	//store curDat to archive
	curDat.timestamp = QTime::currentTime();
	dataSeries.push_back(curDat);
	++count;

	//update Plot	//when time allows, so enqueue in event loop
	QTimer::singleShot(0, this, SLOT(updatePlot()));
//	updatePlot();
}

void Datacenter::updatePlot(void) {
	// add the latest data to the plot
	//TODO check if this is really efficient, but works for the moment
	memmove(climbRateData, climbRateData + 1,
			(plotDataSize - 1) * sizeof(double));
	memmove(elevatorData, elevatorData + 1,
			(plotDataSize - 1) * sizeof(double));

	climbRateData[plotDataSize - 1] = curDat.vh_ind;
//	climbRateData[plotDataSize - 1] = curDat.true_theta;	//for pitch control

	elevatorData[plotDataSize - 1] = curDat.artstab_pitch_ratio;
	climbCurve->setSamples(timeData, climbRateData, plotDataSize);
	elevatorCurve->setSamples(timeData, elevatorData, plotDataSize);

	memmove(rollRateData, rollRateData + 1,
			(plotDataSize - 1) * sizeof(double));
	memmove(aileronData, aileronData + 1, (plotDataSize - 1) * sizeof(double));

	rollRateData[plotDataSize - 1] = curDat.true_phi;
	aileronData[plotDataSize - 1] = curDat.artstab_roll_ratio;
	rollCurve->setSamples(timeData, rollRateData, plotDataSize);
	aileronCurve->setSamples(timeData, aileronData, plotDataSize);

	emit replotSpeedCurve();
	emit replotRollCurve();

}

void Datacenter::receiverBeaconCallback(
		XPlaneBeaconListener::XPlaneServer server, bool exists) {
	std::cout << "receiverBeaconCallback got [" << server.toString() << " is "
			<< (exists ? "alive" : "dead") << "]\n";
	host = server.host;
	port = server.receivePort;

	setConnected(exists);
}

void Datacenter::receiverCallbackFloat(std::string dataref, float value) {
	//Add the value to the hashmap of datarefs
	//TODO welche Daten wollen wir jetzt wie und wo speichern?
//	receivedData[dataref] = {value, time(NULL)};
	switch (hash(dataref.c_str())) {
	case hash("sim/flightmodel/position/indicated_airspeed"):
		curDat.indicated_airspeed = value;
		break;
	case hash("sim/flightmodel/position/true_airspeed"):
		curDat.true_airspeed = value;
		break;
	case hash("sim/flightmodel/position/groundspeed"):
		curDat.groundspeed = value;
		break;

	case hash("sim/flightmodel/position/vh_ind_fpm"):
		curDat.vh_ind_fpm = value;
		break;
	case hash("sim/flightmodel/position/vh_ind"):
		curDat.vh_ind = value;
		curDat.counter++;	//this is one of the controlled parameters
		break;
	case hash("sim/flightmodel/position/vpath"):
		curDat.vpath = value;
		break;
	case hash("sim/flightmodel/position/alpha"):
		curDat.alpha = value;
		break;
	case hash("sim/flightmodel/position/true_theta"):
		curDat.true_theta = value;
		break;
	case hash("sim/flightmodel/position/true_phi"):
		curDat.true_phi = value;
		curDat.counter++;	//this is one of the controlled parameters
		break;
	case hash("sim/flightmodel/position/true_psi"):
		curDat.true_psi = value;
		break;

	case hash("sim/flightmodel/position/elevation"):
		curDat.setPos_WGS84(curDat.getPosition_WGS84().lati,
				curDat.getPosition_WGS84().longi, value);
		break;
	case hash("sim/flightmodel/position/latitude"):
		curDat.setPos_WGS84(value, curDat.getPosition_WGS84().longi,
				curDat.getPosition_WGS84().height);
		break;
	case hash("sim/flightmodel/position/longitude"):
		curDat.setPos_WGS84(curDat.getPosition_WGS84().lati, value,
				curDat.getPosition_WGS84().height);
		break;
	case hash("sim/cockpit2/gauges/indicators/altitude_ft_pilot"):
		curDat.altitude_ft_pilot = value;
		break;
	case hash("sim/flightmodel/position/y_agl"):
		curDat.y_agl = value;
		break;

//	case hash("sim/flightmodel2/controls/pitch_ratio"):
//		curDat.pitch = value;
//		break;
//	case hash("sim/flightmodel2/controls/roll_ratio"):
//		curDat.bank = value;
//		break;
	case hash("sim/cockpit2/gauges/indicators/roll_electric_deg_pilot"):
		curDat.roll_electric_deg_pilot = value;
		break;
//	case hash("sim/cockpit2/gauges/indicators/pitch_AHARS_deg_pilot"):
//		curDat.speedKias = value;
//		break;
//	case hash("sim/flightmodel2/misc/AoA_angle_degrees"):
//		curDat.speedKias = value;
//		break;
	default:
		break;
	}
}

void Datacenter::receiverCallbackString(std::string dataref,
		std::string value) {
	std::cout << "receiverCallbackString got [" << dataref << "] and [" << value
			<< "]" << std::endl;
}

void Datacenter::connectToXPlane() {
	//XXX This does not work with the debugger. check why!
	XPlaneBeaconListener::getInstance()->registerNotificationCallback(
			[this](XPlaneBeaconListener::XPlaneServer server,
					bool exists) {return receiverBeaconCallback(server, exists);});
	XPlaneBeaconListener::getInstance()->setDebug(0);
}

void Datacenter::on_qpd_requested_getPosition() {
	emit sigSocketSendData(std::string("message"), curDat.getPosition_WGS84().asJson());
}

void Datacenter::on_qpd_requested_requested_getPositionXY() {
	emit sigSocketSendData(std::string("message"), curDat.getPosition_Cart().asJson());
}

void Datacenter::on_qpd_requested_getOrigin() {
}

void Datacenter::on_qpd_requested_getPlaneState() {
}

void Datacenter::setConnected(bool connected) {
	if (connected != this->connected) {
		this->connected = connected;
		emit XPlaneConnectionChanged(connected);
	}
	//now create a proper XPlaneUDPClient
	if (connected && !xp) {
		std::cout << std::endl << "Found server " << host << ":" << port
				<< std::endl;
		std::cout << "Connect to the XPlane server for the first time"
				<< std::endl;
		xp = new XPlaneUDPClient(host, port,
				[this](std::string dataref, float value) {
					return receiverCallbackFloat(dataref, value);
				}, [this](std::string dataref, std::string value) {
					return receiverCallbackString(dataref, value);});
		xp->setDebug(0);

		xp->subscribeDataRef("sim/flightmodel/position/indicated_airspeed", 20);
		xp->subscribeDataRef("sim/flightmodel/position/true_airspeed", 20);
		xp->subscribeDataRef("sim/flightmodel/position/groundspeed", 20);

		xp->subscribeDataRef("sim/flightmodel/position/vh_ind_fpm", 20);
		xp->subscribeDataRef("sim/flightmodel/position/vh_ind", 20);

		xp->subscribeDataRef("sim/flightmodel/position/vpath", 20);
		xp->subscribeDataRef("sim/flightmodel/position/alpha", 20);
		xp->subscribeDataRef("sim/flightmodel/position/true_theta", 20);
		xp->subscribeDataRef("sim/flightmodel/position/true_phi", 20);
		xp->subscribeDataRef("sim/flightmodel/position/true_psi", 20);

		xp->subscribeDataRef("sim/flightmodel/position/elevation", 20);
		xp->subscribeDataRef("sim/flightmodel/position/latitude", 20);
		xp->subscribeDataRef("sim/flightmodel/position/longitude", 20);
		xp->subscribeDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot",
				20);
		xp->subscribeDataRef("sim/flightmodel/position/y_agl", 20);

//		xp->subscribeDataRef("sim/flightmodel2/controls/pitch_ratio", 1);
//		xp->subscribeDataRef("sim/flightmodel2/controls/roll_ratio", 1);
		xp->subscribeDataRef(
				"sim/cockpit2/gauges/indicators/roll_electric_deg_pilot", 20);
//		xp->subscribeDataRef("sim/cockpit2/gauges/indicators/pitch_AHARS_deg_pilot", 20);
		curDat.counter = 0;
	}

	//we only need the timer, while being connected to XPlane
	if (connected && xp) {
		std::cout << "started timer" << std::endl;
		emit attachSpeedCurve(climbCurve);
		emit attachSpeedCurve(elevatorCurve);
		std::cout << "SpeedCurve attached\n";
//		emit attachElevatorCurve(elevatorCurve);
		emit attachRollCurve(rollCurve);
		emit attachRollCurve(aileronCurve);
		std::cout << "RollCurve attached\n";

		//TODO der Timer sollte nicht mehr gebraucht werden, wenn XPlane alles schön kann
		basicTimer->start(timerMilliseconds);
	} else {
		basicTimer->stop();
	}
}


