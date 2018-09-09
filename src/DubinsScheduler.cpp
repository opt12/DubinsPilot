/*
 * DubinsScheduler.cpp
 *
 *  Created on: 13.07.2018
 *      Author: eckstein
 */

#include <DubinsScheduler.h>
#include <iostream>
#include "utils.h"

class QTimer;

DataCenter *DubinsScheduler::dc = DataCenter::getInstance();

DubinsScheduler::DubinsScheduler(QObject* parent) :
		QObject(parent) {
	flyPathTimer = new QTimer();
	connect(flyPathTimer, SIGNAL(timeout()), this, SLOT(flyThePath()));
}

DubinsScheduler::~DubinsScheduler() {
	// free the entries in flightPhases Vector
	clearOutSchedule();
}

void DubinsScheduler::takeMeDown(bool active) {
	if (active) {
		// TODO das wäre deutlich besser, hier einfach die Zielpositiion anzugeben und den Pfadtyp
		// TODO und dann halt selber rechnen. Dann kann der Dubins Pfad sogar lokal bleiben
		initialize(DataCenter::getInstance()->getCurrentDubinsPath());	// XXX
		flyPathTimer->start(timerMilliseconds);
		emit sigPathTrackingStatus(true);
		std::cout << "Start Path tracking...\n";
		dc->changeSegmentStatisticsState(false);//just to perform a proper reset of stats
	} else {
		std::cout << "Stop Path tracking...\n";
		emit sigCtrlActiveStateChanged(ctrlType::RADIUS_CONTROL, false);
		emit sigPathTrackingStatus(false);
		dc->changeSegmentStatisticsState(false);//just to perform a proper reset of stats
		flyPathTimer->stop();
	}
}

//TODO Hier muss ich noch die Korrektur der Koordinaten des Dubins-Pfades für Translation gegenüber Earth-Frame implementieren.
// es sollte ausreichen den gesamten Wind-Frame zu verschieben, so wie ich das beim Circle-Fly mache
// Dann auch ein Update über Socket schicken
void DubinsScheduler::flyThePath(void) {
	if (dc->isSimulationPaused()) {
		return;	//we just skip everything, when the simulation is Paused
	}

	if (currentPhase < flightPhases.size()) {
		flightPhases[currentPhase]->performFlight();
		if (flightPhases[currentPhase]->isFinished()) {
			json j;
			const Position* target;
			double targetHeading;
			switch (currentPhase) {
			case 0:	//Circle In
				std::cout << "Finished Phase: Circle In:\n";
				j["flightPhase"] = "Circle In";
				pathFollowStats[currentPhase]->setName("Circle In");
				target = &(db->getCircleExit())[0];
				targetHeading = db->getCircleExitHeading(0);
				break;
			case 1:	//straight Segment Tangential
				j["flightPhase"] = "Straight Tangential";
				pathFollowStats[currentPhase]->setName("Straight Tangential");
				std::cout << "Finished Phase: straight Segment Tangential:\n";
				target = &(db->getCircleEntry())[1];
				targetHeading = db->getCircleExitHeading(0);
				break;
			case 2:	//Circle Out
				j["flightPhase"] = "Circle Out";
				pathFollowStats[currentPhase]->setName("Circle Out");
				std::cout << "Finished Phase: Circle Out:\n";
				target = &(db->getCircleExit())[1];
				targetHeading = db->getCircleExitHeading(1);
				break;
			case 3:	//straight Segment FinalApproach // EndPosition
				j["flightPhase"] = "Straight Final Approach";
				pathFollowStats[currentPhase]->setName("Final Approach");
				std::cout
						<< "Finished Phase: straight Segment FinalApproach:\n";
				target = &db->getEndPoint();
				targetHeading = db->getCircleExitHeading(1);
				break;
			default: //runOutPhase
				j["flightPhase"] = "Run Out";
				pathFollowStats[currentPhase]->setName("Run Out");
				std::cout << "Finished Phase: runOutPhase:\n";
				target = &db->getEndPoint();
				targetHeading = db->getCircleExitHeading(1);
				break;
			}

			//update all the stats
			const SegmentStatistics * segStat = dc->getSegmentStats();
//			j["segmentStats"] = segStat->asJson();
			pathFollowStats[currentPhase]->cartesianPathLengthEarthFrame =
					segStat->getCartesianPathLengthEarthFrame();
			pathFollowStats[currentPhase]->cartesianPathLengthWindFrame =
					segStat->getCartesianPathLengthWindFrame();
			pathFollowStats[currentPhase]->flightTime =
					segStat->getFlightTime();
			pathFollowStats[currentPhase]->heightLoss =
					segStat->getHeightLoss();
			pathFollowStats[currentPhase]->glideAngleEarthFrame =
					to_degrees(
							atan(
									pathFollowStats[currentPhase]->heightLoss
											/ pathFollowStats[currentPhase]->cartesianPathLengthEarthFrame));
			pathFollowStats[currentPhase]->glideAngleWindFrame =
					to_degrees(
							atan(
									pathFollowStats[currentPhase]->heightLoss
											/ pathFollowStats[currentPhase]->cartesianPathLengthWindFrame));
			pathFollowStats[currentPhase]->totalAngleFlown =
					segStat->getTotalAngleFlown();
			pathFollowStats[currentPhase]->totalWindDisplacement =
					segStat->getWindDisplacement();

			Position currentPos = dc->getPosition();
			pathFollowStats[currentPhase]->errorEarthFrame =
					target->getCartesianDifference(currentPos);
			Position targetWindFrame = *target
					+ (dc->getWindDisplacement() - initialDisplacement);
			pathFollowStats[currentPhase]->errorWindFrame =
					targetWindFrame.getCartesianDifference(currentPos);
			pathFollowStats[currentPhase]->headingError = getAngularDifference(
					dc->getTrue_psi(), targetHeading);

			j["segmentError"]["PositionEarthFrame"] =
					pathFollowStats[currentPhase]->errorEarthFrame.asJson();
			j["segmentError"]["PositionWindFrame"] =
					pathFollowStats[currentPhase]->errorWindFrame.asJson();
			j["headingError"] = pathFollowStats[currentPhase]->headingError;

			if( debug ){
				std::cout << "\tFinished Phase " << j["flightPhase"]
						<< " with stats:\n" << j.dump(4) << "\n";
				std::cout << "\tEntering Flight Phase " << currentPhase
						<< " now.\n\n";
			}

			currentPhase++;
		}
	} else {
		std::cout
				<< "I'm already there. No safe spot to get off the rotten plane?\n";
		emit sigCtrlActiveStateChanged(ctrlType::RADIUS_CONTROL, false);
		emit sigPathTrackingStatus(false);
		flyPathTimer->stop();
		statsSummaryAsJson();
		std::cout <<"Statistics:\n"<< statistics.dump(4) <<std::endl;
		emit sigOutputPathTrackingStats(statistics);
		if(statistics["flownPath"]["earthFrame"]["windDisplacement"]["Length"]>EPS){
			emit sigSocketSendData(std::string("DUBINS_PATH_BLOWN"), 0,
					db->blownAsJson(pathFollowStats[0]->totalWindDisplacement,
							pathFollowStats[1]->totalWindDisplacement,
							pathFollowStats[2]->totalWindDisplacement,
							pathFollowStats[3]->totalWindDisplacement));
		}
		//this stops the world after reaching the target if selected in GUI
		emit sigPauseSimTriggered(true);
	}
}

void DubinsScheduler::clearOutSchedule(void) {
	// we need to clear the flight phases vector and free its memory, before we start with a new schedule
	// free the entries in flightPhases Vector
	for (auto fp : flightPhases) {
		// disconnect the signals
		QObject::disconnect(fp,
				SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)), this,
				SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)));
		QObject::disconnect(fp,
				SIGNAL(
						sigRequestedSetValueChanged(ctrlType, double, bool, bool)),
				this,
				SIGNAL(
						sigRequestedSetValueChanged(ctrlType, double, bool, bool)));
		QObject::disconnect(fp, SIGNAL(sigCircleDirectionChanged(bool, double)),
				this, SIGNAL(sigCircleDirectionChanged(bool, double)));
		QObject::disconnect(fp, SIGNAL(sigDisplayFlightPhase(QString, QString)),
				this, SIGNAL(sigDisplayFlightPhase(QString, QString)));
		free(fp);//don't have to delete the container itself, as it resides on the stack
	}
	flightPhases.clear();
}

void DubinsScheduler::clearOutStats(void) {
	for (auto st : pathFollowStats) {
		free(st);
	}
	pathFollowStats.clear();
	statistics = json({});
}

json DubinsScheduler::statsSummaryAsJson(void) {
	double cartesianPathLengthEarthFrame = 0.0, cartesianPathLengthWindFrame =
			0.0;
	double cartesianPathLengthEarthFrameStraight = 0.0,
			cartesianPathLengthWindFrameStraight = 0.0;
	double cartesianPathLengthEarthFrameCircle = 0.0,
			cartesianPathLengthWindFrameCircle = 0.0;
	double flightTime = 0.0;
	double heightLoss = 0.0, heightLossStraight = 0.0, heightLossCircle = 0.0;
	Position_Cartesian totalWindDisplacement = Position_Cartesian();
	double totalAngleFlown = 0.0;
	double glideAngleEarthFrameStraight = 0.0, glideAngleEarthFrameCircle = 0.0;
	double glideAngleWindFrameStraight = 0.0, glideAngleWindFrameCircle = 0.0;

	statistics["segStats"] = json::array();
	for (auto st : pathFollowStats) {
		if(st->segType!= +segmentTypeEnum::INVALID){	//no need to include run out phase
			statistics["segStats"].push_back(st->asJson());
		}
		cartesianPathLengthEarthFrame += st->cartesianPathLengthEarthFrame;
		cartesianPathLengthWindFrame += st->cartesianPathLengthWindFrame;
		if ((+segmentTypeEnum::L == st->segType)
				|| (+segmentTypeEnum::R == st->segType)) {
			cartesianPathLengthEarthFrameCircle +=
					st->cartesianPathLengthEarthFrame;
			cartesianPathLengthWindFrameCircle +=
					st->cartesianPathLengthWindFrame;
			heightLossCircle += st->heightLoss;
		} else {
			cartesianPathLengthEarthFrameStraight +=
					st->cartesianPathLengthEarthFrame;
			cartesianPathLengthWindFrameStraight +=
					st->cartesianPathLengthEarthFrame;
			heightLossStraight += st->heightLoss;
		}
		flightTime += st->flightTime;
		heightLoss += st->heightLoss;
		totalWindDisplacement = totalWindDisplacement
				+ st->totalWindDisplacement;
		totalAngleFlown += st->totalAngleFlown;
	}

	glideAngleEarthFrameStraight = to_degrees(
			atan(heightLossStraight / cartesianPathLengthEarthFrameStraight));
	glideAngleEarthFrameCircle = to_degrees(
			atan(heightLossCircle / cartesianPathLengthEarthFrameCircle));
	glideAngleWindFrameStraight = to_degrees(
			atan(heightLossStraight / cartesianPathLengthWindFrameStraight));
	glideAngleWindFrameCircle = to_degrees(
			atan(heightLossCircle / cartesianPathLengthWindFrameCircle));

	json j;
	j["airFrame"]["pathLength"] = cartesianPathLengthWindFrame;
	j["airFrame"]["pathLengthDetails"]["pathLengthCircleIn"] = pathFollowStats[0]->cartesianPathLengthWindFrame;
	j["airFrame"]["pathLengthDetails"]["pathLengthStraightTangential"] = pathFollowStats[1]->cartesianPathLengthWindFrame;
	j["airFrame"]["pathLengthDetails"]["pathLengthCircleOut"] = pathFollowStats[2]->cartesianPathLengthWindFrame;
	j["airFrame"]["pathLengthDetails"]["pathLengthStraightFinal"] = pathFollowStats[3]->cartesianPathLengthWindFrame;
	j["earthFrame"]["pathLength"] = cartesianPathLengthEarthFrame;
	j["earthFrame"]["pathLengthDetails"]["pathLengthCircleIn"] = pathFollowStats[0]->cartesianPathLengthEarthFrame;
	j["earthFrame"]["pathLengthDetails"]["pathLengthStraightTangential"] = pathFollowStats[1]->cartesianPathLengthEarthFrame;
	j["earthFrame"]["pathLengthDetails"]["pathLengthCircleOut"] = pathFollowStats[2]->cartesianPathLengthEarthFrame;
	j["earthFrame"]["pathLengthDetails"]["pathLengthStraightFinal"] = pathFollowStats[3]->cartesianPathLengthEarthFrame;

	j["totalAngle"] = totalAngleFlown;
	j["angleDetails"]["angleCircleIn"] = pathFollowStats[0]->totalAngleFlown;
	j["angleDetails"]["angleCircleOut"] = pathFollowStats[2]->totalAngleFlown;


	j["airFrame"]["glideAngle"]["glideAngleStraight"] = glideAngleWindFrameStraight;
	j["airFrame"]["glideAngle"]["glideAngleCircle"] = glideAngleWindFrameCircle;
	j["earthFrame"]["glideAngle"]["glideAngleStraight"] = glideAngleEarthFrameStraight;
	j["earthFrame"]["glideAngle"]["glideAngleCircle"] = glideAngleEarthFrameCircle;

	j["heightLoss"] = heightLoss;
	j["heightLossDetails"]["timeCircleIn"] = pathFollowStats[0]->heightLoss;
	j["heightLossDetails"]["timeStraightTangential"] = pathFollowStats[1]->heightLoss;
	j["heightLossDetails"]["timeCircleOut"] = pathFollowStats[2]->heightLoss;
	j["heightLossDetails"]["timeStraightFinal"] = pathFollowStats[3]->heightLoss;

	j["flightTime"] = flightTime;
	j["flightTimeDetails"]["timeCircleIn"] = pathFollowStats[0]->flightTime;
	j["flightTimeDetails"]["timeStraightTangential"] = pathFollowStats[1]->flightTime;
	j["flightTimeDetails"]["timeCircleOut"] = pathFollowStats[2]->flightTime;
	j["flightTimeDetails"]["timeStraightFinal"] = pathFollowStats[3]->flightTime;

	j["earthFrame"]["windDisplacement"]["Length"] = totalWindDisplacement.length();
	j["earthFrame"]["windDisplacement"]["Heading"] = totalWindDisplacement.heading();

	j["earthFrame"]["deviation"] = pathFollowStats[3]->errorEarthFrame.asJson();
	j["earthFrame"]["deviation"]["heading"] = pathFollowStats[3]->headingError;
	j["airFrame"]["deviation"] = pathFollowStats[3]->errorWindFrame.asJson();
	j["airFrame"]["deviation"]["heading"] = pathFollowStats[3]->headingError;

	std::cout << "PathTracking Statistical Summary:\n";
//	std::cout << j.dump(4) << std::endl;
	statistics["flownPath"]=j;
	return j;
}

json DubinsScheduler::dubinsPathCartesiansAsJson(void) {
	// This puts out the relevant Points of the Dubins path as JSON object
	json j;

	// Reference is the entry point of the first circle, as the plane position is here
	Position reference = db->getStartPoint();
	Position_Cartesian diff = reference.getCartesianDifference(
			db->getStartPoint());
	j["Start_Point"] = diff.asJson();

	diff = reference.getCartesianDifference(db->getCircleEntry()[0]);
	j["Circle_0_Entry"] = diff.asJson();
	diff = reference.getCartesianDifference(db->getCircleCenter()[0]);
	j["Circle_0_Center"] = diff.asJson();
	diff = reference.getCartesianDifference(db->getCircleExit()[0]);
	j["Circle_0_Exit"] = diff.asJson();

	diff = reference.getCartesianDifference(db->getCircleEntry()[1]);
	j["Circle_1_Entry"] = diff.asJson();
	diff = reference.getCartesianDifference(db->getCircleCenter()[1]);
	j["Circle_1_Center"] = diff.asJson();
	diff = reference.getCartesianDifference(db->getCircleExit()[1]);
	j["Circle_1_Exit"] = diff.asJson();

	diff = reference.getCartesianDifference(db->getEndPoint());
	j["End_Point"] = diff.asJson();

	Position_Cartesian disp = Position_Cartesian(0, db->getCircleRadius(), 0);
	disp = disp.rotate(db->getCircleExitAngle()[1]);
	diff = reference.getCartesianDifference(db->getEndPoint() + disp);
	j["Virtual_EndCircle"] = diff.asJson();

	return j;
}

void DubinsScheduler::initialize(const DubinsPath* _db) {
	db = _db;
	pathTypeEnum pathType = db->getPathType();

	clearOutSchedule();
	clearOutStats();

//	std::cout << dubinsPathCartesiansAsJson().dump(4) << std::endl;
	std::cout << "Start Path tracking for:\n";
	statistics["plannedPath"]=db->dubinsPathCharacteristicsAsJson();
	std::cout << statistics["plannedPath"].dump(4) << std::endl;

	statistics["pathSpecs"]=db->dubinsPathSpecificationAsJson();
	std::cout << statistics["pathSpecs"].dump(4) << std::endl;


	currentPhase = 0;

//	Position current = DataCenter::getInstance()->getPosition();
	initialDisplacement = dc->getWindDisplacement();//we need this for the statistics;

	flightPhases.push_back(
	// first circle segment
			new CirclePhase(db->getCircleCenter()[0], db->getCircleRadius(),
					db->getCircleEntryAngle()[0], db->getCircleExitAngle()[0]));// path type R or L is set later on
	flightPhases.push_back(
	// tangent between circles
			new StraightPhase(db->getCircleExit()[0], db->getCircleEntry()[1],
					db->getCircleRadius() / 4, db->getCircleCenter()[1]));
	flightPhases.push_back(
	// second circle segment
			new CirclePhase(db->getCircleCenter()[1], db->getCircleRadius(),
					db->getCircleEntryAngle()[1], db->getCircleExitAngle()[1]));// path type R or L is set later on
	flightPhases.push_back(
	// final approach straight
			new StraightPhase(db->getCircleExit()[1], db->getEndPoint(),
					db->getCircleRadius(),//longer for the final approach is better for short FA tracks
					db->getEndPoint()
							+ Position_Cartesian(0, db->getCircleRadius(), 0).rotate(
									db->getCircleExitAngle()[1])));
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
		flightPhases[1]->setSegmentType(segmentTypeEnum::S);
		flightPhases[2]->setSegmentType(segmentTypeEnum::L);
		flightPhases[3]->setSegmentType(segmentTypeEnum::FINAL);
		flightPhases[4]->setSegmentType(segmentTypeEnum::INVALID);
		break;
	case pathTypeEnum::RSR:
		flightPhases[0]->setSegmentType(segmentTypeEnum::R);
		flightPhases[1]->setSegmentType(segmentTypeEnum::S);
		flightPhases[2]->setSegmentType(segmentTypeEnum::R);
		flightPhases[3]->setSegmentType(segmentTypeEnum::FINAL);
		flightPhases[4]->setSegmentType(segmentTypeEnum::INVALID);
		break;
	}

	for (auto fp : flightPhases) {
		//add statistics objects to the vector;
		pathFollowStats.push_back(new PathFollowStats());
		pathFollowStats.back()->segType = fp->getSegType();
		//now we have to forward the signals from the flightPhases to the DubinsScheduler
		QObject::connect(fp, SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)),
				this, SIGNAL(sigCtrlActiveStateChanged(ctrlType, bool)));
		QObject::connect(fp,
				SIGNAL(
						sigRequestedSetValueChanged(ctrlType, double, bool, bool)),
				this,
				SIGNAL(
						sigRequestedSetValueChanged(ctrlType, double, bool, bool)));
		QObject::connect(fp, SIGNAL(sigCircleDirectionChanged(bool, double)),
				this, SIGNAL(sigCircleDirectionChanged(bool, double)));
		QObject::connect(fp, SIGNAL(sigDisplayFlightPhase(QString, QString)),
				this, SIGNAL(sigDisplayFlightPhase(QString, QString)));
	}
}

json DubinsScheduler::PathFollowStats::asJson(void){
	json j;
	j["segmentName"] = segmentName;
	j["earthFrame"]["pathLength"] = cartesianPathLengthEarthFrame;
	j["airFrame"]["pathLength"] = cartesianPathLengthWindFrame;
	j["earthFrame"]["heightLoss"] = heightLoss;
	j["airFrame"]["heightLoss"] = heightLoss;
	j["windDisplacement"] = totalWindDisplacement.asJson();
	j["angleFlown"] = totalAngleFlown;
	j["earthFrame"]["glideAngle"] = glideAngleEarthFrame;
	j["airFrame"]["glideAngle"] = glideAngleWindFrame;
	j["flightTime"]=flightTime;
	j["airFrame"]["deviation"] = errorWindFrame.asJson();
	j["airFrame"]["deviation"]["heading"] = headingError;
	j["earthFrame"]["deviation"] = errorEarthFrame.asJson();
	j["earthFrame"]["deviation"]["heading"] = headingError;

	return j;
}

