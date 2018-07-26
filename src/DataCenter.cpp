/*
 * Datacenter.cpp
 *
 *  Created on: 05.05.2018
 *      Author: eckstein
 */
#include <DataCenter.h>
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
#include <qtextstream.h>
#include <qfile.h>
#include "constants.h"
#include "utils.h"

constexpr
static unsigned int hash(const char* str, int h = 0) { //TODO check if this hashing is a good idea...
													   //https://stackoverflow.com/questions/16388510/evaluate-a-string-with-a-switch-in-c/16388610#16388610
	return !str[h] ? 5381 : (hash(str, h + 1) * 33) ^ str[h];
}

//helper function to get the sign of a number in a typesafe manner
template<typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

// initialize statics
DataCenter * DataCenter::instance = NULL;
int DataCenter::UPDATE_RATE = 20;

DataCenter::DataCenter(QObject* parent) :
		QObject(parent) {

//	curDat.climbControlActive = false;
//	curDat.rollControlActive = false;

	basicTimer = new QTimer();
	QObject::connect(basicTimer, SIGNAL(timeout()), this, SLOT(timerExpired()));
	QObject::connect(this, SIGNAL(sigCalculateDubinsPath(pathTypeEnum)), this,
			SLOT(calculateDubinsPath(pathTypeEnum)));

	//TODO XXX comment out to Debug
	connectToXPlane();//TODO Das funktioniert nicht immer zuverlässig. Was ist da los?
//	QTimer::singleShot(0, this, setConnected(true));	//only enqueue this into the event queue
}

DataCenter::~DataCenter() {
	if (xp) {
		xp->setDataRef("sim/operation/override/override_artstab", false);
		xp->setDataRef("sim/operation/override/override_joystick_pitch", false);
		xp->setDataRef("sim/operation/override/override_joystick_roll", false);
	}
	basicTimer->stop();
	free(basicTimer);
}

//Private Slots:
void DataCenter::timerExpired(void) {
	//TODO Das ist nicht schön, aber da ich den setConnected() Slot nicht per singleShot Timer aufrufen kann ist das die einfachste Lösung.
	//TODO Die connection zu XPlane läuft noch nicht so, wie es sein soll. Er erkennt aktuell keinen Abriss
	emit XPlaneConnectionChanged(connected);

	if (loggingActive) {
		*outLog << curDat.csvDataRecord();
//		std::cout << curDat.csvDataRecord().toStdString();
	}

	//store curDat to archive
	curDat.timestamp = QTime::currentTime();
	dataSeries.push_back(curDat);//copies the curDat and is hence save to use here

	//integrate over the windDisplacement
	// when the direction is updated, I also update the integration over the wind displacement
	curDat.windDisplacement.x -= sin(to_radians(curDat.wind_direction_degt))
			* curDat.wind_speed_ms / (1000 / timerMilliseconds);
	curDat.windDisplacement.y -= cos(to_radians(curDat.wind_direction_degt))
			* curDat.wind_speed_ms / (1000 / timerMilliseconds);
//	{static int count = 0;
//	if(!(count%(1000/timerMilliseconds)))
//			std::cout << "timestamp: "<< QTime::currentTime().toString("hh:mm:ss.zzz").toStdString().c_str()<<";\twindDisplacement: "
//					<< curDat.windDisplacement.asJson().dump(4)
//					<< std::endl;
//	count++;
//	}

	//send the entire Dataset out to the ipc-socket The node.js app can listen if it wants to
	//no need to implement a question answer protocol
	//TODO das sollte wirklich alle 100msec raus und nicht nur jede Sekunde
	if (!(count % 1)) {
		on_rqd_requested_getPlaneState(0);
	}

	++count;
}

void DataCenter::receiverBeaconCallback(
		XPlaneBeaconListener::XPlaneServer server, bool exists) {
	std::cout << "receiverBeaconCallback got [" << server.toString() << " is "
			<< (exists ? "alive" : "dead") << "]\n";
	host = server.host;
	port = server.receivePort;

	//TODO ist das besser, wenn ich das in die queue einreihe? Das klappt irgendwie nicht ganz immer
	setConnected(exists);
}

void DataCenter::receiverCallbackFloat(std::string dataref, float value) {
	//Add the value to the hashmap of datarefs
	//TODO welche Daten wollen wir jetzt wie und wo speichern?
//	receivedData[dataref] = {value, time(NULL)};
	switch (hash(dataref.c_str())) {
	case hash("sim/flightmodel/position/indicated_airspeed"):
		curDat.indicated_airspeed_ms = knots_to_ms(value);
		break;
	case hash("sim/flightmodel/position/true_airspeed"):
		curDat.true_airspeed = value; //this is one of the controlled parameters
		break;
	case hash("sim/flightmodel/position/groundspeed"):
		curDat.groundspeed = value;
		break;
	case hash("sim/flightmodel/position/vh_ind_fpm"):
		curDat.vh_ind_fpm = value;
		break;
	case hash("sim/flightmodel/position/vh_ind"):
		curDat.vh_ind = value;	//this is one of the controlled parameters
		curDat.counter++;
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
	case hash("sim/cockpit2/gauges/indicators/roll_electric_deg_pilot"):
		curDat.roll_electric_deg_pilot = value;
		break;
	case hash("sim/weather/wind_direction_degt"):
		if (fabs(curDat.wind_direction_degt - value) >= 0.1) {
			emit sigWindChanged(value, curDat.wind_speed_ms);
		}
		curDat.wind_direction_degt = value;
		break;
	case hash("sim/weather/wind_speed_kt"):
		if (fabs(curDat.wind_speed_ms - value) >= 0.1) {
			// XXX In contrast to the docs, wind speed is given directly in m/s instead of knots;
			emit sigWindChanged(curDat.wind_direction_degt,
					(value <= 0.1) ? 0.0 : value);
		}
		// XXX In contrast to the docs, wind speed is given directly in m/s instead of knots;
		curDat.wind_speed_ms = value;
		break;
	default:
		break;
	}
}

void DataCenter::receiverCallbackString(std::string dataref,
		std::string value) {
	std::cout << "receiverCallbackString got [" << dataref << "] and [" << value
			<< "]" << std::endl;
}

void DataCenter::connectToXPlane() {
	//XXX This does not work with the debugger. check why!
	XPlaneBeaconListener::getInstance()->setDebug(1);
	XPlaneBeaconListener::getInstance()->registerNotificationCallback(
			[this](XPlaneBeaconListener::XPlaneServer server,
					bool exists) {return receiverBeaconCallback(server, exists);});
}

//void DataCenter::setRequestedAltitude(double altitudeAboveGround) {
//	if (xp) {
//		xp->setDataRef("sim/flightmodel/position/local_y", altitudeAboveGround);
//		std::cout << "Altitude set to " << altitudeAboveGround << " meters"
//				<< std::endl;
//	}
//}
//

void DataCenter::on_rqd_requested_getPosition(int requestId) {
	emit sigSocketSendData(std::string("POSITION"), requestId,
			curDat.getPosition_WGS84().asJson());
}

void DataCenter::on_rpd_requested_getPositionXY(int requestId) {
	emit sigSocketSendData(std::string("POSITION_XY"), requestId,
			curDat.getPosition_Cart().asJson());
}

void DataCenter::on_rqd_requested_getOrigin(int requestId) {
}

void DataCenter::on_rqd_requested_getPlaneState(int requestId) {
	emit sigSocketSendData(std::string("PLANE_STATE"), requestId,
			curDat.asJson());
}

void DataCenter::setElfLocation(double forward, double right, double height,
		double rotation, pathTypeEnum pathType) {
	//we don't necessarily need the origin set to define an ELF location
	//we only need it when starting to calculate the path to do this in XY cartesian coords
	curDat.setElfPos_relative(curDat.pos, forward, right, height, // the height cannot be set directly here
			rotation); 	//relative to current position

	double minHeightLoss = DubinsPath::calculateMinimumHeightLoss(curDat.pos,
			curDat.elf, curDat.true_psi, curDat.elfHeading, CIRCLE_RADIUS,
			GLIDE_RATIO_STRAIGHT, GLIDE_RATIO_CIRCLE, pathType);

	if (height == 0.0) {
		curDat.setElfHeightDiff(minHeightLoss);
		curDat.elf.setHeight(
				curDat.pos.getPosition_WGS84().height - minHeightLoss);
	} else if (height < 0.0) {
		curDat.elf.setHeight(curDat.pos.getPosition_WGS84().height - (-height));
	} else if (height > 0.0) {
		curDat.elf.setHeight(height);
	}

	if (curDat.elf.getPosition_WGS84().height < 0.0
			|| ((curDat.pos.getPosition_WGS84().height
					- curDat.elf.getPosition_WGS84().height)
					< (minHeightLoss - EPS))) {
		// we won't make it there. the height loss is inconsistent
		// TODO negative elevations are neglected :-(
		std::cout
				<< "ATTENTION!!! The given height of the ELf is not feasible; We're not gonna make it there!\n";
		// The real failure handling is done in calculating the path
	}

	//implicit else
	std::cout << "ELF determined to be"
			<< curDat.elf.getPosition_WGS84().asJson().dump(4) << std::endl;
	std::cout << "ELF heading set to " << curDat.elfHeading << std::endl;
	std::cout << "ELF distance from here is "
			<< curDat.pos.getDistanceCart(curDat.elf)
			<< ", Height difference is: "
			<< curDat.pos.getPosition_WGS84().height
					- curDat.elf.getPosition_WGS84().height << std::endl;
	std::cout << "minimum height loss is " << minHeightLoss;
	std::cout << ", heading is " << curDat.pos.getHeadingCart(curDat.elf)
			<< std::endl;
	std::cout << "Now try to calculate the concrete path;\n";

//	emit sigElfCoordsSet(curDat.elf.getPosition_WGS84(), curDat.elfHeading, true);
	emit sigCalculateDubinsPath(pathType);
	json j;
	if(curDat.isElfSet){
		j["position"] = curDat.elf.getPosition_WGS84().asJson();
		j["heading"] = curDat.elfHeading;
		j["isElfSet"] = curDat.isElfSet;
	}
	emit sigSocketSendData(std::string("ELF_POSITION"), 0, j);

	emit sigCalculateDubinsPath(pathType);
}

void DataCenter::setElfLocation(Position_WGS84 elfPosition, double elfHeading,
		pathTypeEnum pathType) {
	curDat.setElfPos(elfPosition, elfHeading);
	emit sigCalculateDubinsPath(pathType);
	json j;
	if(curDat.isElfSet){
		j["position"] = curDat.elf.getPosition_WGS84().asJson();
		j["heading"] = curDat.elfHeading;
		j["isElfSet"] = curDat.isElfSet;
	}
	emit sigSocketSendData(std::string("ELF_POSITION"), 0, j);
}

void DataCenter::resetElfLocation(void) {
	curDat.resetElfPos();
	emit sigSocketSendData(std::string("DUBINS_PATH"), 0, json::object( { }));
	emit sigSocketSendData(std::string("ELF_POSITION"), 0, json::object( { }));
}

void DataCenter::calculateDubinsPath(pathTypeEnum pathType) {
	if (!curDat.isElfSet) {
		std::cout << "You need to specify the ELF first\n";
		return;
	}
//	std::cout<< "Calculating dubinsPath now! \n";
	//now calculate the dubins Path from the current Position to the desired ELF
	//TODO of course, we need to check the dubins Path Parameters like
	// RSR, LSL, glideAngle, radius...
	curDat.setDubinsPath(curDat.pos, curDat.elf, curDat.true_psi,
			curDat.elfHeading, CIRCLE_RADIUS, GLIDE_RATIO_STRAIGHT,
			GLIDE_RATIO_CIRCLE, pathType);

	//TODO ist das sinnvoll? ist ja eigentlich egal
//	curDat.isElfSet = curDat.isValidDubinsPath();	// in case it's invalid, we got to reset

	emit sigElfCoordsSet(curDat.elf.getPosition_WGS84(), curDat.elfHeading,
			curDat.isValidDubinsPath());
	emit sigSocketSendData(std::string("DUBINS_PATH"), 0, curDat.db.asJson());
}

void DataCenter::setOrigin(void) {
	Position_WGS84 pos = curDat.getPosition_WGS84();
	Position::setOrigin(pos);
	originSet = true;
	std::cout << "Origin set to: "
			<< Position::getOrigin_WGS84().asJson().dump(4) << std::endl;
	if (outLog) {
		//insert the new Origin into the logfile, if it exists
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
	}
	emit originSetTo(Position::getOrigin_WGS84());
}

void DataCenter::resetWindDisplacement(void) {
	curDat.resetWindDisplacement();
}

void DataCenter::invokeLogging(bool active, QFile* fileLog) {
	loggingActive = active;
	if (active) {
		outLog = new QTextStream(fileLog);
		if (!originSet) {
			setOrigin();
			resetWindDisplacement();
		}
		*outLog << Dataset::csvHeading();

	} else {
		free(outLog);
	}
	std::cout << "loggingActive: " << loggingActive << "; File: "
			<< fileLog->fileName().toStdString() << std::endl;
}

void DataCenter::SendXPDataRef(const char* dataRefName, double value) {
	if (xp) {
		xp->setDataRef(dataRefName, value);
	}
}

void DataCenter::SendXPDataRef(const char* dataRefName, bool state) {
	if (xp) {
		xp->setDataRef(dataRefName, state);
	}
}

void DataCenter::setConnected(bool connected) {
	if (connected != this->connected) {
		this->connected = connected;
		//TODO Die connection zu XPlane läuft noch nicht so, wie es sein soll. Er erkennt aktuell keinen Abriss
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
					return receiverCallbackString(dataref, value);
				});
		xp->setDebug(0);

		xp->subscribeDataRef("sim/flightmodel/position/indicated_airspeed",
				UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/true_airspeed",
				UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/groundspeed",
				UPDATE_RATE);

		xp->subscribeDataRef("sim/flightmodel/position/vh_ind_fpm",
				UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/vh_ind", UPDATE_RATE);

		xp->subscribeDataRef("sim/flightmodel/position/vpath", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/alpha", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/true_theta",
				UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/true_phi", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/true_psi", UPDATE_RATE);

		xp->subscribeDataRef("sim/flightmodel/position/elevation", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/latitude", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/longitude", UPDATE_RATE);
		xp->subscribeDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot",
				UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/y_agl", UPDATE_RATE);
		xp->subscribeDataRef(
				"sim/cockpit2/gauges/indicators/roll_electric_deg_pilot",
				UPDATE_RATE);
		// XXX at least the wind is not emitted with the subscribed update rate!!!
		xp->subscribeDataRef("sim/weather/wind_direction_degt", UPDATE_RATE);
		xp->subscribeDataRef("sim/weather/wind_speed_kt", UPDATE_RATE);

		curDat.counter = 0;
	}

	//we only need the timer, while being connected to XPlane
	if (connected && xp) {
		std::cout << "started timer" << std::endl;
		//TODO der Timer sollte nicht mehr gebraucht werden, wenn XPlane alles schön kann
		basicTimer->start(timerMilliseconds);
	} else {
		basicTimer->stop();
	}
}

