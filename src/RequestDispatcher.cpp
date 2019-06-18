/*
 * RequestDispatcher.cpp
 *
 *  Created on: 01.06.2018
 *      Author: eckstein
 */

#include "RequestDispatcher.h"
#include <iostream>

constexpr
static unsigned int hash(const char* str, int h = 0) { //TODO check if this hashing is a good idea...
													   //https://stackoverflow.com/questions/16388510/evaluate-a-string-with-a-switch-in-c/16388610#16388610
	return !str[h] ? 5381 : (hash(str, h + 1) * 33) ^ str[h];
}

RequestDispatcher::RequestDispatcher(QObject* parent) :
		QObject(parent) {

}

RequestDispatcher::~RequestDispatcher() {
	// TODO Auto-generated destructor stub
}

void RequestDispatcher::dispatchSockMessage(json receivedJson) {
	std::string msgType;
	try {
		msgType = receivedJson.value("type", "malformedRequest");
	} catch (const std::exception& e) {
		msgType = "malformedRequest";
		std::cout << "Reception Error while getting data from Socket\n";
		std::cout << e.what() << std::endl;
	}
	int requestId = -1;
	json data;
	try {
		data = receivedJson["data"];
		requestId = receivedJson["requestId"];
	} catch (const std::exception& e) {
		std::cout << "Malformed request received from Socket\n";
		std::cout << e.what() << std::endl;;
	}

	if (debug) {
		std::cout << "Received Request:\n" << receivedJson.dump(4);
	}

	switch (hash(msgType.c_str())) {
//TODO Prüfe, ob das alles nicht einfach rauskann, weil ich es aktuell nicht benutze
//	case hash("GET_POSITION"):
//		emit requested_getPosition(requestId);
//		break;
//	case hash("GET_POSITION_XY"):
//		emit requested_getPositionXY(requestId);
//		break;
//	case hash("GET_ORIGIN"):
//		emit requested_getOrigin(requestId);
//		break;
//	case hash("GET_PLANE_STATE"):
//		emit requested_getPlaneState(requestId);
//		break;
	case hash("SET_PLANE_STATE"):
		std::cout << "SET_PLANE_STATE received with data:\n";
		std::cout << data.dump(4) << std::endl;
		emit sigSetPlaneState(data, requestId);
		break;
	case hash("SET_ELEVATOR"):
		std::cout << "SET_ELEVATOR received with data:\n";
		std::cout << data.dump(4) << std::endl;
		try {
			emit sigSendGymControl("sim/joystick/yoke_pitch_ratio", data["yoke_pitch_ratio"]);
		} catch (const std::exception& e) {
			std::cout << "SET_ELEVATOR: " << e.what() << std::endl;
		}
		break;

	case hash("malformedRequest"):
	default:
		std::cout << " please include a proper \"type\"-field and a \"requestId\"-field"
		" in and request\n";
		break;
	}

}
