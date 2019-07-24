/*
 * Datacenter.cpp
 *
 *  Created on: 05.05.2018
 *      Author: eckstein
 */
#include <DataCenter.h>
#include <pid_controller.h>

#include <QObject>
#include <QFileInfo>

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
	QObject::connect(this, SIGNAL(XPlaneConnectionChanged(bool)), this,
			SLOT(setConnectionState(bool)));

}

DataCenter::~DataCenter() {
	if (xp) {
		xp->setDataRef("sim/operation/override/override_artstab", false);
		xp->setDataRef("sim/operation/override/override_joystick_pitch", false);
		xp->setDataRef("sim/operation/override/override_joystick_roll", false);
	}
	basicTimer->stop();
	delete basicTimer;
}

//Private Slots:
void DataCenter::timerExpired(void) {
	//TODO Das ist nicht schön, aber da ich den setConnected() Slot nicht per singleShot Timer aufrufen kann ist das die einfachste Lösung.
	//TODO Die connection zu XPlane läuft noch nicht so, wie es sein soll. Er erkennt aktuell keinen Abriss
	emit XPlaneConnectionDisplayUI(currentConnection.isConnected);

	if(DataCenter::getInstance()->isSimulationPaused()){
		return;	//we just skip everything, when the simulation is Paused
	}


	if (loggingActive) {
		*outLog << curDat.csvDataRecord();
	}

	//store curDat to archive
	curDat.timestamp = QTime::currentTime();
	// the dataSeries may be used for in memory storage of all data, but long term it's a waste of memory.
	//	dataSeries.push_back(curDat);

	//integrate over the windDisplacement
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
	emit sigSocketSendData(std::string("PLANE_STATE"), 0,
				curDat.asJson());

	++count;
}

void DataCenter::receiverBeaconCallback(
		XPlaneBeaconListener::XPlaneServer server, bool exists) {
	std::cout << "receiverBeaconCallback got [" << server.toString() << " is "
			<< (exists ? "alive" : "dead") << "]\n";

	if (exists){
		if(!currentConnection.isConnected){
			//we are not connected, but there is something to connect
			currentConnection.host = server.host;
			currentConnection.port = server.receivePort;
			currentConnection.name = server.name;

			std::cout << "connected to " << currentConnection.name << " ("
					<< currentConnection.host << ":" << currentConnection.port << ")\n";

			emit XPlaneConnectionChanged(exists);
		} else {
			//some other connection got lost
		}
	} else {
		if(!exists && currentConnection.host == server.host && currentConnection.port == server.receivePort){
			//we lost our connection
			emit XPlaneConnectionChanged(exists);
		}
	}
}

void DataCenter::receiverCallbackFloat(std::string dataref, float value) {
	//Add the value to the hashmap of datarefs
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
		curDat.true_psi = value;	//this is one of the controlled parameters
		break;
	case hash("sim/flightmodel/position/elevation"):
		curDat.setPos_WGS84(curDat.getPosition_WGS84().lati,
				curDat.getPosition_WGS84().longi, value);
		break;
	case hash("sim/flightmodel/position/latitude"):
		curDat.setPos_WGS84(value, curDat.getPosition_WGS84().longi,
				curDat.getPosition_WGS84().altitude);
		break;
	case hash("sim/flightmodel/position/longitude"):
		curDat.setPos_WGS84(curDat.getPosition_WGS84().lati, value,
				curDat.getPosition_WGS84().altitude);
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
	case hash("sim/flightmodel/position/local_vx"):
		curDat.local_vx = value;
		break;
	case hash("sim/flightmodel/position/local_vy"):
		curDat.local_vy = value;
		break;
	case hash("sim/flightmodel/position/local_vz"):
		curDat.local_vz = value;
		break;
	case hash("sim/flightmodel/position/local_y"):
		curDat.local_y = value;
		break;
	case hash("sim/flightmodel/position/q[0]"):
		curDat.rotationQuaternion[0] = value;
		break;
	case hash("sim/flightmodel/position/q[1]"):
		curDat.rotationQuaternion[1] = value;
		break;
	case hash("sim/flightmodel/position/q[2]"):
		curDat.rotationQuaternion[2] = value;
		break;
	case hash("sim/flightmodel/position/q[3]"):
		curDat.rotationQuaternion[3] = value;
		break;
	case hash("sim/flightmodel/failures/stallwarning"):
		curDat.stallWarning = value;
		break;
	case hash("sim/flightmodel/misc/h_ind"):
		curDat.h_ind = value;
		break;
	case hash("sim/joystick/yoke_pitch_ratio"):
		curDat.yoke_pitch_ratio = value;
		break;
	case hash("sim/joystick/yoke_roll_ratio"):
		curDat.yoke_roll_ratio = value;
		break;
	case hash("sim/joystick/yoke_heading_ratio"):
		curDat.yoke_heading_ratio = value;
		break;
	case hash("sim/flightmodel/position/Qrad"):
		curDat.Qrad = value;
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
	XPlaneBeaconListener::getInstance()->setDebug(true);
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

//TODO Prüfe, ob das alles nicht einfach rauskann, weil ich es aktuell nicht benutze
//void DataCenter::on_rqd_requested_getPosition(int requestId) {
//	emit sigSocketSendData(std::string("POSITION"), requestId,
//			curDat.getPosition_WGS84().asJson());
//}
//
//void DataCenter::on_rpd_requested_getPositionXY(int requestId) {
//	emit sigSocketSendData(std::string("POSITION_XY"), requestId,
//			curDat.getPosition_Cart().asJson());
//}
//
//void DataCenter::on_rqd_requested_getOrigin(int requestId) {
//}
//
//void DataCenter::on_rqd_requested_getPlaneState(int requestId) {
//	emit sigSocketSendData(std::string("PLANE_STATE"), requestId,
//			curDat.asJson());
//}

void DataCenter::setElfLocation(double forward, double right, double height,
		double rotation, pathTypeEnum pathType) {
	//we don't necessarily need the origin set to define an ELF location
	//we only need it when starting to calculate the path to do this in XY cartesian coords
	curDat.setElfPos_relative(curDat.pos, forward, right, height, // the height cannot be set directly here
			rotation); 	//relative to current position

	double minHeightLoss = DubinsPath::calculateMinimumHeightLoss(curDat.pos,
			curDat.elf, curDat.true_psi, curDat.elfHeading, autoCircleRadius,
			autoGlideRatioCircle, autoGlideRatioStraight, pathType);

	if (height == 0.0) {
		curDat.setElfHeightDiff(minHeightLoss);
		curDat.elf.setAltitude(
				curDat.pos.getPosition_WGS84().altitude - minHeightLoss);
	} else if (height < 0.0) {
		curDat.elf.setAltitude(curDat.pos.getPosition_WGS84().altitude - (-height));
	} else if (height > 0.0) {
		curDat.elf.setAltitude(height);
	}

	if (curDat.elf.getPosition_WGS84().altitude < 0.0
			|| ((curDat.pos.getPosition_WGS84().altitude
					- curDat.elf.getPosition_WGS84().altitude)
					< (minHeightLoss - EPS))) {
		// we won't make it there. the height loss is inconsistent
		// TODO negative elevations are neglected :-(
		std::cout
				<< "ATTENTION!!! The given height of the ELf is not feasible; We're not gonna make it there!\n";
		std::cout << "Minimum height loss is minHeightLoss >= "
				<< minHeightLoss << "\n";
		// The real failure handling is done in calculating the path
	}

	//implicit else
	if (debug) {
		std::cout << "ELF determined to be"
				<< curDat.elf.getPosition_WGS84().asJson().dump(4) << std::endl;
		std::cout << "ELF heading set to " << curDat.elfHeading << std::endl;
		std::cout << "ELF distance from here is "
				<< curDat.pos.getDistanceCart(curDat.elf)
				<< ", Height difference is: "
				<< curDat.pos.getPosition_WGS84().altitude
						- curDat.elf.getPosition_WGS84().altitude << std::endl;
		std::cout << "minimum height loss is " << minHeightLoss;
		std::cout << ", heading is " << curDat.pos.getHeadingCart(curDat.elf)
				<< std::endl;
		std::cout << "Now try to calculate the concrete path;\n";
	}
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
	emit sigSocketSendData(std::string("DUBINS_PATH_BLOWN"), 0, json::object( { }));
	emit sigSocketSendData(std::string("ELF_POSITION"), 0, json::object( { }));
}

void DataCenter::changeSegmentStatisticsState(bool isActive){
	if(!isActive){
		segmentStats.stopSegmentStatsUpdate();
//		std::cout << "Stats of last segment are: " << segmentStats.asJson().dump(4) << "\n";
//		std::cout<<"changeSegmentStatisticsState(false)\n";
	} else {
		if(!segmentStats.isSegmentStatsActive()){
//			std::cout<<"changeSegmentStatisticsState(true)\n";
			segmentStats.initialize();
		} else {
			//nothing to do, it's already running;
		}
	}
}

void DataCenter::updateSegmentStatistics(void){
	segmentStats.updateSegmentData();
}

void DataCenter::setFlightPathCharacteristics(double valClimbRateStraight,
		double valClimbRateCircle, double valAutoCircleRadius){
	autoGlideAngleStraight = valClimbRateStraight;
	autoGlideRatioStraight = 1/tan(deg2rad(autoGlideAngleStraight));
	autoGlideAngleCircle = valClimbRateCircle;
	autoGlideRatioCircle = 1/tan(deg2rad(autoGlideAngleCircle));
	autoCircleRadius = valAutoCircleRadius;
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
			curDat.elfHeading, autoCircleRadius, autoGlideRatioCircle,
			autoGlideRatioStraight, pathType);

	//TODO ist das sinnvoll? ist ja eigentlich egal
//	curDat.isElfSet = curDat.isValidDubinsPath();	// in case it's invalid, we got to reset

	emit sigElfCoordsSet(curDat.elf.getPosition_WGS84(), curDat.elfHeading,
			curDat.isValidDubinsPath());
	emit sigSocketSendData(std::string("DUBINS_PATH"), 0, curDat.db.asJson());
	emit sigSocketSendData(std::string("DUBINS_PATH_BLOWN"), 0, json::object( { }));
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
				+ QString::number(curDat.getPosition_WGS84().altitude, 'g', 6)
				+ ";\n";
		*outLog << origin;
	}
	emit originSetTo(Position::getOrigin_WGS84());
}

void DataCenter::resetWindDisplacement(void) {
	curDat.resetWindDisplacement();
}

//void DataCenter::invokeLogging(bool active, QFile* fileLog) {
//	loggingActive = active;
//	if (active) {
//		outLog = new QTextStream(fileLog);
//		if (!originSet) {
//			setOrigin();
//			resetWindDisplacement();
//		}
//		*outLog << Dataset::csvHeading();
//
//	} else {
//		delete outLog;
//	}
//	std::cout << (loggingActive?"Started ":"Stopped ") << "logging; File: "
//			<< fileLog->fileName().toStdString() << std::endl;
//}

void DataCenter::invokeLogging(bool active, QDir _logFileDir, QString _fileName) {
	if(active){
		//start logging
		logFileDir = _logFileDir;
		logFileName = _fileName;
		if(!QDir(logFileDir).exists()){
			if(!QDir().mkdir(logFileDir.absolutePath())){
				//cannot create that directory
				std::cout << "Cannot create directory: "
						<< logFileDir.absolutePath().toStdString() << std::endl;
				emit sigLoggingStateChanged(false);
				return;
			}
		}
		QFileInfo logFileInfo = QFileInfo(_logFileDir, _fileName);
		fileLog = new QFile();
		fileLog->setFileName(logFileInfo.absoluteFilePath());
		if (!fileLog->open(QIODevice::WriteOnly)) {
			std::cout << "Cannot open file for writing: "
					<< qPrintable(fileLog->errorString()) << std::endl;
			emit sigLoggingStateChanged(false);
			delete fileLog;
			return;
		}
		loggingActive = true;
		std::cout << "loggingActive: " << loggingActive << "; File: "
				<< fileLog->fileName().toStdString() << std::endl;
		outLog = new QTextStream(fileLog);
		setOrigin(); //we do this with each logfile
//		resetWindDisplacement();
		*outLog << Dataset::csvHeading();
		emit sigLoggingStateChanged(true);
	} else {
		//stop logging
		loggingActive = false;
		if(outLog != NULL){
			outLog->flush();
			delete outLog;
			outLog = NULL;
		}
		if(fileLog->isOpen()){
			fileLog->close();
		}
		std::cout << (loggingActive?"Started ":"Stopped ") << "logging; File: "
				<< fileLog->fileName().toStdString() << std::endl;
		if(fileLog != NULL){
			delete fileLog;
			fileLog = NULL;
		}
		emit sigLoggingStateChanged(false);
	}
}

void DataCenter::outputPathTrackingStats(const json stats) {
	emit sigSocketSendData("PATH_TRACKING_STATS", 0, stats);	//send data to the socket

	if(loggingActive){
		//try to open a file and dump the JSON in;
		QString baseName = QFileInfo(logFileName).completeBaseName();
		baseName = baseName+".JSON";
		QFileInfo jsonFileInfo = QFileInfo(logFileDir, baseName);
		QFile jsonFile;
		jsonFile.setFileName(jsonFileInfo.absoluteFilePath());
		if (!jsonFile.open(QIODevice::WriteOnly)) {
			std::cout << "Cannot open file for writing: "
					<< qPrintable(jsonFile.errorString()) << std::endl;
			return;
		}
		QTextStream outJson(&jsonFile);
		stats.dump(4);
		jsonFile.write(stats.dump(4).c_str());
		outJson.flush();
		jsonFile.close();
	} else {
		std::cout<<"Logging not active\n";
	}
}


void DataCenter::SendXPDataRef(const char* dataRefName, double value) {
	if (currentConnection.isConnected) {
		xp->setDataRef(dataRefName, value);
	}
}

void DataCenter::SendXPDataRef(const char* dataRefName, bool state) {
	if (currentConnection.isConnected) {
		xp->setDataRef(dataRefName, state);
	}
}

void DataCenter::setConnectionState(bool connected) {
	if (currentConnection.isConnected && !connected) {
		//we lost the connection
		currentConnection.isConnected = false;
		delete xp;
		std::cout << "Lost Connection to " << currentConnection.name << " on "
				<< currentConnection.host << ":" << currentConnection.port
				<< std::endl;
		currentConnection.name = "";
		currentConnection.host = "";
		currentConnection.port = 0;
	} else if (!currentConnection.isConnected && connected) {
		//a new connection needs to be established

		//now create a proper XPlaneUDPClient
		std::cout << std::endl << "Found server " << currentConnection.name << "; IP-address is "
				<< currentConnection.host << ":" << currentConnection.port
				<< std::endl;
		std::cout << "Connect to the XPlane server for the first time"
				<< std::endl;
		xp = new XPlaneUDPClient(currentConnection.host, currentConnection.port,
				[this](std::string dataref, float value) {
					return receiverCallbackFloat(dataref, value);
				}, [this](std::string dataref, std::string value) {
					return receiverCallbackString(dataref, value);
				});
		xp->setDebug(false);

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

		xp->subscribeDataRef("sim/flightmodel/position/q[0]", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/q[1]", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/q[2]", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/q[3]", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/local_y", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/local_vx", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/local_vy", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/position/local_vz", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/failures/stallwarning", UPDATE_RATE);
		xp->subscribeDataRef("sim/flightmodel/misc/h_ind", UPDATE_RATE);
		xp->subscribeDataRef("sim/joystick/yoke_pitch_ratio", UPDATE_RATE);
		xp->subscribeDataRef("sim/joystick/yoke_roll_ratio", UPDATE_RATE);
		xp->subscribeDataRef("sim/joystick/yoke_heading_ratio", UPDATE_RATE);

		curDat.counter = 0;
		currentConnection.isConnected = true;
	}

	emit XPlaneConnectionDisplayUI(currentConnection.isConnected);

	//we only need the timer, while being connected to XPlane
	if (currentConnection.isConnected) {
		basicTimer->start(timerMilliseconds);
		if (debug) {
			std::cout << "started timer" << std::endl;
		}
	} else {
		basicTimer->stop();
	}
}

