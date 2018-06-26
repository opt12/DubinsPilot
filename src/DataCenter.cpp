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

DataCenter::DataCenter(QObject* parent) :
		QObject(parent) {

//	curDat.climbControlActive = false;
//	curDat.rollControlActive = false;

	basicTimer = new QTimer();
	QObject::connect(basicTimer, SIGNAL(timeout()), this, SLOT(timerExpired()));

	//TODO XXX comment out to Debug
	connectToXPlane();
//	QTimer::singleShot(0, this, setConnected(true));	//only enqueue this into the event queue
}

DataCenter::~DataCenter() {
	// TODO Auto-generated destructor stub
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
	if (loggingActive) {
		*outLog << curDat.csvDataRecord();
//		std::cout << curDat.csvDataRecord().toStdString();
	}

	//store curDat to archive
	curDat.timestamp = QTime::currentTime();
	dataSeries.push_back(curDat);	//copies the curDat and is hence save to use here

	//send the entire Dataset out to the ipc-socket The node.js app can listen if it wants to
	//no need to implement a question answer protocol
	//TODO das sollte wirklich alle 100msec raus und nicht nur jede Sekunde
	if(!(count%1)){
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

	setConnected(exists);
}

void DataCenter::receiverCallbackFloat(std::string dataref, float value) {
	//Add the value to the hashmap of datarefs
	//TODO welche Daten wollen wir jetzt wie und wo speichern?
//	receivedData[dataref] = {value, time(NULL)};
	switch (hash(dataref.c_str())) {
	case hash("sim/flightmodel/position/indicated_airspeed"):
		curDat.indicated_airspeed = value;
		break;
//	case hash("sim/flightmodel/position/indicated_airspeed2"):
//		curDat.indicated_airspeed2 = value;
//		break;
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
	case hash("sim/weather/wind_direction_degt"):
		curDat.wind_direction_degt = value;
		break;
	case hash("sim/weather/wind_speed_kt"):
		curDat.wind_speed_kt = value;
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
	XPlaneBeaconListener::getInstance()->registerNotificationCallback(
			[this](XPlaneBeaconListener::XPlaneServer server,
					bool exists) {return receiverBeaconCallback(server, exists);});
	XPlaneBeaconListener::getInstance()->setDebug(0);
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
	emit sigSocketSendData(std::string("POSITION"),
			requestId,
			curDat.getPosition_WGS84().asJson());
}

void DataCenter::on_rpd_requested_getPositionXY(int requestId) {
	emit sigSocketSendData(std::string("POSITION_XY"),
			requestId,
			curDat.getPosition_Cart().asJson());
}

void DataCenter::on_rqd_requested_getOrigin(int requestId) {
}

void DataCenter::on_rqd_requested_getPlaneState(int requestId) {
		emit sigSocketSendData(std::string("PLANE_STATE"),
				requestId,
				curDat.asJson());
}

void DataCenter::setElfLocation(double forward, double right, double height, double rotation){
	//we don't necessarily need the origin set to define an ELF file
	//we only need it when starting to calculate the path to do this in XY cartesian coords
	curDat.setElfPos_relative(forward, right, height, rotation);	//relative to current position
	std::cout << "ELF set to "<< curDat.elf.getPosition_WGS84().asJson().dump(4)<< std::endl;
	std::cout << "ELF heading set to " << curDat.elfHeading << std::endl;
	std::cout << "ELF distance from here is " << curDat.pos.getDistanceCart(curDat.elf);
	std::cout << ", heading is "<< curDat.pos.getHeadingCart(curDat.elf) << std::endl;


	emit sigElfCoordsSet(curDat.elf.getPosition_WGS84(),
			curDat.elf.getPosition_Cart(), curDat.elfHeading);
}

void DataCenter::setOrigin(void){
	Position_WGS84 pos =  curDat.getPosition_WGS84();
	Position::setOrigin(pos);
	originSet = true;
	std::cout <<"Origin set to: " << Position::getOrigin_WGS84().asJson().dump(4) << std::endl;
	if(outLog){
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
	emit sigElfCoordsSet(curDat.elf.getPosition_WGS84(),
			curDat.elf.getPosition_Cart(), curDat.elfHeading);
}

void DataCenter::invokeLogging(bool active, QFile* fileLog) {
	loggingActive = active;
	if (active) {
		outLog = new QTextStream(fileLog);
		if(! originSet){
			setOrigin();
		}
		*outLog << Dataset::csvHeading();

	} else {
		free(outLog);
	}
	std::cout << "loggingActive: " << loggingActive << "; File: "
			<< fileLog->fileName().toStdString() << std::endl;
}

void DataCenter::SendXPDataRef(const char* dataRefName, double value){
		if (xp) {
			xp->setDataRef(dataRefName, value);
		}
}

void DataCenter::SendXPDataRef(const char* dataRefName, bool state){
	if (xp) {
		xp->setDataRef(dataRefName, state);
	}
}



void DataCenter::setConnected(bool connected) {
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
				},
				[this](std::string dataref, std::string value) {
					return receiverCallbackString(dataref, value);
				});
		xp->setDebug(0);

		xp->subscribeDataRef("sim/flightmodel/position/indicated_airspeed", 20);
//		xp->subscribeDataRef("sim/flightmodel/position/indicated_airspeed2", 20);
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
		xp->subscribeDataRef("sim/weather/wind_direction_degt", 20);
		xp->subscribeDataRef("sim/weather/wind_speed_kt", 20);

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

