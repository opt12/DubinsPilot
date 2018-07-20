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

void DubinsScheduler::takeMeDown(bool active){
	if(active){
	// TODO das wäre deutlich besser, hier einfach die Zielpositiion anzugeben und den Pfadtyp
	// TODO und dann halt selber rechnen. Dann kann der Dubins Pfad sogar lokal bleiben
	initialize(DataCenter::getInstance()->getCurrentDubinsPath());	// XXX
	flyPathTimer->start(timerMilliseconds);
	emit sigPathTrackingStatus(true);
	std::cout << "Start Path tracking...\n";
	} else {
		std::cout << "Stop Path tracking...\n";
		emit sigCtrlActiveStateChanged(ctrlType::RADIUS_CONTROL, false);
		emit sigPathTrackingStatus(false);
		flyPathTimer->stop();
	}
}


void DubinsScheduler::flyThePath(void) {
	if(currentPhase < flightPhases.size()){
		flightPhases[currentPhase]->performCommand();
		if(flightPhases[currentPhase]->isFinished()){
			currentPhase++;
			std::cout <<"\tEntering Flight Phase "<< currentPhase<< " now.\n\n";
		}
	} else {
		std::cout <<"I'm already there. No safe spot to get off the rotten plane?\n";
		emit sigCtrlActiveStateChanged(ctrlType::RADIUS_CONTROL, false);
		//TODO entscheide, ob nach Abbruch die richtung beibehalten wird,
//		emit sigCtrlActiveStateChanged(ctrlType::HEADING_CONTROL, true);
		emit sigPathTrackingStatus(false);
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
		QObject::disconnect(fp, SIGNAL(sigRequestedSetValueChanged(ctrlType, double, bool, bool)),
				this, SIGNAL(sigRequestedSetValueChanged(ctrlType, double, bool, bool)));
		QObject::disconnect(fp, SIGNAL(sigCircleDirectionChanged(bool, double)),
				this, SIGNAL(sigCircleDirectionChanged(bool, double)));
		QObject::disconnect(fp, SIGNAL(sigDisplayFlightPhase(QString, QString)),
						this, SIGNAL(sigDisplayFlightPhase(QString, QString)));
		free(fp);	//don't have to delete the container itself, as it resides on the stack
	}
	flightPhases.clear();
}


void DubinsScheduler::initialize(const DubinsPath* _db) {
	db = _db;
	pathTypeEnum pathType = db->getPathType();

	clearOutSchedule();

	currentPhase = 0;

	Position current = DataCenter::getInstance()->getPosition();

	flightPhases.push_back(
			// first circle segment
			new CirclePhase(db->getCircleCenter()[0], db->getCircleRadius(),
					db->getCircleEntryAngle()[0],
					db->getCircleExitAngle()[0]));
	flightPhases.push_back(
			// tangent between circles
			new StraightPhase(db->getCircleExit()[0], db->getCircleEntry()[1],
					db->getCircleRadius(), db->getCircleCenter()[1]));
	flightPhases.push_back(
			// second circle segment
			new CirclePhase(db->getCircleCenter()[1], db->getCircleRadius(),
					db->getCircleEntryAngle()[1],
					db->getCircleExitAngle()[1]));
	flightPhases.push_back(
			// final approach straight
			new StraightPhase(db->getCircleExit()[1], db->getEndPoint(),
					db->getCircleRadius(),
					db->getEndPoint() +
							Position_Cartesian(0, db->getCircleRadius(),0)
									.rotate(90)));
	flightPhases.push_back(
			// we are already there, but no touchdown.
			new RunOutPhase(db->getEndPoint(),
					(db->getPathType() == +pathTypeEnum::LSL
							|| db->getPathType() == +pathTypeEnum::RSL) ?
							db->getCircleExitAngle()[1] - 90 :
							db->getCircleExitAngle()[1] + 90));

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
		QObject::connect(fp, SIGNAL(sigRequestedSetValueChanged(ctrlType, double, bool, bool)),
				this, SIGNAL(sigRequestedSetValueChanged(ctrlType, double, bool, bool)));
		QObject::connect(fp, SIGNAL(sigCircleDirectionChanged(bool, double)),
						this, SIGNAL(sigCircleDirectionChanged(bool, double)));
		QObject::connect(fp, SIGNAL(sigDisplayFlightPhase(QString, QString)),
						this, SIGNAL(sigDisplayFlightPhase(QString, QString)));
	}
}
