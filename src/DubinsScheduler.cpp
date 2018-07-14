/*
 * DubinsScheduler.cpp
 *
 *  Created on: 13.07.2018
 *      Author: eckstein
 */

#include <DubinsScheduler.h>
#include <iostream>
class QTimer;

DubinsScheduler::DubinsScheduler(QObject* parent) :
		QObject(parent) {
	flyPathTimer = new QTimer();
	connect(flyPathTimer, SIGNAL(timeout()), this, SLOT(flyThePath()));
}

DubinsScheduler::~DubinsScheduler() {
	// free the entries in flightPhases Vector
	clearOutSchedule();
}

void DubinsScheduler::flyThePath(void) {
	static unsigned int currentPhase = 0;
	if(currentPhase < flightPhases.size()){
		flightPhases[currentPhase]->performCommand();
		if(flightPhases[currentPhase]->isFinished()){
			std::cout <<"\tEntering Flight Phase "<< currentPhase<< "now.\n\n";
			currentPhase++;
		}
	} else {
		std::cout <<"I'm already there. No safe spot to get off the rotten plane?\n";
		// TODO emit some signal to stop the pathFollowing
		flyPathTimer->stop();
	}
}

void DubinsScheduler::clearOutSchedule(void){
	// we need to clear the flight phases vector and free its memory, before we start with a new schedule
	// free the entries in flightPhases Vector
	for(auto fp : flightPhases){
		// disconnect the signals
		QObject::disconnect(fp, SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)),
				this, SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)));
		QObject::disconnect(fp, SIGNAL(sigRequestedSetValueChanged(ctrlType, double)),
				this, SIGNAL(sigRequestedSetValueChanged(ctrlType, double)));
		QObject::disconnect(fp, SIGNAL(sigCircleDirectionChanged(bool, double)),
				this, SIGNAL(sigCircleDirectionChanged(bool, double)));

		free(fp);	//don't have to delete the container itself, as it resides on the stack
	}
	flightPhases.clear();
}


void DubinsScheduler::initialize(const DubinsPath* _db) {
	db = _db;
	pathTypeEnum pathType = db->getPathType();

	clearOutSchedule();

	flightPhases.push_back(
			new CirclePhase(db->getCircleCenter()[0], db->getCircleRadius(),
					db->getCircleEntryAngle()[0],
					db->getCircleExitAngle()[0]));
	flightPhases.push_back(
			new StraightPhase(db->getCircleExit()[0], db->getCircleEntry()[1],
					db->getCircleRadius(), db->getCircleCenter()[1]));
	flightPhases.push_back(
			new CirclePhase(db->getCircleCenter()[1], db->getCircleRadius(),
					db->getCircleEntryAngle()[1],
					db->getCircleExitAngle()[1]));
	flightPhases.push_back(
			new StraightPhase(db->getCircleExit()[1], db->getEndPoint(),
					db->getCircleRadius(),
					db->getEndPoint() +
							Position_Cartesian(0, db->getCircleRadius(),0)
									.rotate(90)));
	flightPhases.push_back(
			new FinalApproachPhase(db->getEndPoint(),
					db->getCircleExitAngle()[0] + 90));

	switch (pathType) {	//intentionally, there's no default case, so I get appropriate warnings
	case pathTypeEnum::LSL:
		flightPhases[0]->setSegmentType(segmentTypeEnum::L);
		flightPhases[2]->setSegmentType(segmentTypeEnum::L);
		break;
	case pathTypeEnum::RSR:
		flightPhases[0]->setSegmentType(segmentTypeEnum::R);
		flightPhases[2]->setSegmentType(segmentTypeEnum::R);
		break;
	}

	//now we have to forward the signals from the flightPhases to the DubinsScheduler
	for(auto fp : flightPhases){
		QObject::connect(fp, SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)),
				this, SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)));
		QObject::connect(fp, SIGNAL(sigRequestedSetValueChanged(ctrlType, double)),
				this, SIGNAL(sigRequestedSetValueChanged(ctrlType, double)));
		QObject::connect(fp, SIGNAL(sigCircleDirectionChanged(bool, double)),
				this, SIGNAL(sigCircleDirectionChanged(bool, double)));
	}
}
