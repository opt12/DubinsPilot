/*
 * Gym.cpp
 *
 *  Created on: 03.06.2019
 *      Author: felix
 */

#include <Gym.h>
#include <iostream>

DataCenter *Gym::dc = DataCenter::getInstance();

Gym::Gym(QObject* parent) {
	// TODO Auto-generated constructor stub

}

Gym::~Gym() {
	// TODO Auto-generated destructor stub
}


void Gym::setPlaneState(json j_data, int requestId) {

	for (auto& x : j_data.items())
	{
//	    std::cout << "key: " << x.key() << ", value:" << x.value() << '\n';
		//just pass the incoming values to the X-Plane simulator
		//TODO some validation is highly recommended
		emit sigSendXPDataRef(x.key().c_str(), x.value());
	}
//	emit sigSendXPDataRef("sim/flightmodel/position/P", 0.0);
//	emit sigSendXPDataRef("sim/flightmodel/position/Q", 0.0);
//	emit sigSendXPDataRef("sim/flightmodel/position/R", 0.0);
}

void Gym::setGymControl(const char* key, double actionValue) {
	//TODO disable the autopilot with PID controls and do some plausibility checks
	dc->setControllerOutputs(ctrlType::CLIMB_CONTROL, actionValue);
	emit sigSendXPDataRef(key, actionValue);
}
