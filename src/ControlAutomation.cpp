/*
 * ControlAutomation.cpp
 *
 *  Created on: 01.09.2018
 *      Author: eckstein
 */

#include <ControlAutomation.h>
#include "DataCenter.h"
#include <iostream>

const double ControlAutomation::MAX =  99999.99;
const int ControlAutomation::TIMER_VALUE =  250;

ControlAutomation::ControlAutomation(QObject* parent) :
		QObject(parent) {
	automationTimer = new QTimer();
	dc = DataCenter::getInstance();
	connect(automationTimer, SIGNAL(timeout()), this, SLOT(checkForAutomationEvents()));

	automationTimer->start(TIMER_VALUE);
}


ControlAutomation::~ControlAutomation() {
	// TODO Auto-generated destructor stub
}

void ControlAutomation::setApproachStartingAltitudeChanged(QString approachStartingAltitude) {
	if(approachStartingAltitude.isEmpty()){
		approachStartAltitude = MAX;
		approachStartArmed = false;
		if (debug) {
			std::cout << "Approach Starter disabled\n";
		}
		return;
	}

	approachStartAltitude = ft_to_m(approachStartingAltitude.toDouble());
	if (debug) {
		std::cout << "Approach Starter enabled at "<< approachStartAltitude <<"[m  a.s.l.]\n";
	}

	if(dc->getAltitude()<= approachStartAltitude)
		approachStartArmed = true;
	else
		approachStartArmed = false;
}

void ControlAutomation::setLifeSaverHeightChanged(QString _lifeSaverHeight) {
	if(_lifeSaverHeight.isEmpty()){
		lifeSaverHeight = MAX;
		lifeSaverArmed = false;
		if (debug) {
			std::cout << "Life Saver disabled\n";
		}
		return;
	}

	lifeSaverHeight = ft_to_m(_lifeSaverHeight.toDouble());
	if (debug) {
		std::cout << "Life Saver enabled at "<< lifeSaverHeight <<"[m AGL]\n";
	}
	if(dc->getHeight()<= lifeSaverHeight)
		lifeSaverArmed = true;
	else
		lifeSaverArmed = false;
}

void ControlAutomation::checkForAutomationEvents(void){
	if(dc->isSimulationPaused())
		return;

	if(dc->getAltitude() <= approachStartAltitude){
		if(approachStartArmed){
			approachStartArmed = false;
			std::cout << "Approach Starter triggered at "<< dc->getAltitude() <<"[m  a.s.l.]\n";
			emit sigApproachStartingTriggered();
		}
	} else if(!approachStartArmed){
		approachStartArmed = true;
		if (debug) {
			std::cout << "Approach Starter armed at "<< approachStartAltitude <<"[m  a.s.l.]\n";
		}
	}

	if(dc->getHeight() <= lifeSaverHeight){
		if(lifeSaverArmed){
			lifeSaverArmed = false;
			std::cout << "Life Saver triggered at "<< dc->getHeight() <<"[m  AGL]\n";
			emit sigLifeSaverTriggered();
		}
	} else if(!lifeSaverArmed){
		lifeSaverArmed = true;
		if (debug) {
			std::cout << "Life Saver armed at "<< lifeSaverHeight <<"[m  AGL]\n";
		}
	}
}
