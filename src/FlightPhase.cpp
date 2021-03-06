/*
 * FlightPhase.cpp
 *
 *  Created on: 13.07.2018
 *      Author: eckstein
 */

#include <FlightPhase.h>
#include "utils.h"

#include "json.hpp"
// for convenience
using json = nlohmann::json;
#include <stdio.h>


DataCenter *FlightPhase::dc = DataCenter::getInstance();

FlightPhase::FlightPhase(QObject* parent) :
				QObject(parent) {
	initialDisplacement = dc->getWindDisplacement();
}

FlightPhase::~FlightPhase() {
}




CirclePhase::CirclePhase(Position _circleCenter, double _radius,
		double _angleIn, double _angleOut, QObject* parent) : FlightPhase(parent) {
	circleCenter = _circleCenter;
	radius = _radius;
	angleIn = _angleIn;
	angleOut = _angleOut;
}

CirclePhase::~CirclePhase() {
}

void CirclePhase::performFlight(void) {
	// do I have to leave the circle a little early to compensate
	// the heading overshoot when leaving the circle
	const double ANGULAR_THRESHOLD = 5.0;

	//of course, we need to use the blown circleCenter
	Position circleCenterBlown = circleCenter
			+ (dc->getWindDisplacement() - initialDisplacement);


	double currentAngle = circleCenterBlown.getHeadingCart(dc->getPosition());

	if (!isCircleStarted) {

		dc->changeSegmentStatisticsState(true);

		std::cout <<"Starting circle Flight\n";
		// let's start the circle now by calling the respective Autopilot function
		double distanceToCenter = circleCenterBlown.getDistanceCart(
				dc->getPosition());

		angleFlown = getAngularDifference(currentAngle, angleIn);
		lastAngle = currentAngle;

		if (debug) {
			std::cout <<"angleFlown: "<< angleFlown <<"; angleIn "<< angleIn<<"; angleOut: "<<angleOut<<"\n";
		}

		//we just emit signals to keep loose coupling
		emit sigCircleDirectionChanged((segType == +segmentTypeEnum::L), distanceToCenter);
		emit sigRequestedSetValueChanged(ctrlType::RADIUS_CONTROL, radius, true, (segType == +segmentTypeEnum::L));
		emit sigCtrlActiveStateChanged(ctrlType::RADIUS_CONTROL, true);
		emit sigCtrlActiveStateChanged(ctrlType::CLIMB_CONTROL, true);	//when doing an auto-approach, we want to have all active
		isCircleStarted = true;
		double angleToGo = (segType == +segmentTypeEnum::L) ?
				angleOut - (angleIn + angleFlown) :
				angleOut + (angleIn + angleFlown);
		emit sigDisplayFlightPhase("Circle Flight, "+QString(segType._to_string()),
				"To go: "+QString::number(angleToGo, 'f', 2));
		return;
	}

	// implicit else, we are already in circle flight
	angleFlown += getAngularDifference(lastAngle, currentAngle);
	lastAngle = currentAngle;
	double angleToGo = (segType == +segmentTypeEnum::L) ?
			angleOut - (angleIn + angleFlown) :
			angleOut - (angleIn + angleFlown);
	emit sigDisplayFlightPhase("Circle Flight, "+QString(segType._to_string()),
			"To go: "+QString::number(angleToGo, 'f', 0));

	if ((segType == +segmentTypeEnum::L) ?
			(angleIn + angleFlown) <= angleOut + ANGULAR_THRESHOLD :
			(angleIn + angleFlown) >= angleOut - ANGULAR_THRESHOLD) {
		// we made it through the circle
		emit sigCtrlActiveStateChanged(ctrlType::RADIUS_CONTROL, false);
		isCircleFinished = true;
		dc->changeSegmentStatisticsState(false);	//TODO hier noch die daten der statistik abholen
		std::cout <<"Finishing circle Flight\n";
	}
}

StraightPhase::StraightPhase(Position _start, Position _end,
		double _radiusForOvershoot, Position _nextCircleCenter,
		QObject* parent) : FlightPhase(parent) {

	start = _start;
	end = _end;
	radiusForOvershoot = _radiusForOvershoot;
	nextCircleCenter = _nextCircleCenter;
	tangentialHeading = start.getHeadingCart(end);
	segType = segmentTypeEnum::FINAL;
}

StraightPhase::~StraightPhase() {
}

void StraightPhase::performFlight(void) {
	Position endBlown = end + (dc->getWindDisplacement() - initialDisplacement);
	Position nextcircleCenterBlown = nextCircleCenter
			+ (dc->getWindDisplacement() - initialDisplacement);

//	std::cout<<"WindDisplacement: x: "<< (dc->getWindDisplacement() - initialDisplacement).x
//			<< "; y: " << (dc->getWindDisplacement() - initialDisplacement).y << "\n";

	double targetHeading = dc->getPosition().getHeadingCart(endBlown);
	double currentDistance = dc->getPosition().getDistanceCart(endBlown);
	double currentHeading = dc->getTrue_psi();

	//Localizer Tracking according to Allerton §4.11, p. 189
	double requestedHeading = tangentialHeading- 6*(tangentialHeading -
			targetHeading);

	if(!isStraightStarted){

		dc->changeSegmentStatisticsState(true);

		emit sigCtrlActiveStateChanged(ctrlType::RADIUS_CONTROL, false);	//just to be sure
		emit sigRequestedSetValueChanged(ctrlType::HEADING_CONTROL, requestedHeading, true);
		emit sigCtrlActiveStateChanged(ctrlType::HEADING_CONTROL, true);
		emit sigCtrlActiveStateChanged(ctrlType::CLIMB_CONTROL, true);	//when doing an auto-approach, we want to have all active
		isStraightStarted = true;
		std::cout <<"Straight Flight started\n";
		emit sigDisplayFlightPhase("Straight Flight",
				"Dist: "+QString::number(currentDistance, 'f', 0));
		return;
	}

	// implicit else, we are already in straight flight
	emit sigDisplayFlightPhase("Straight Flight",
			"Dist: "+QString::number(currentDistance, 'f', 0));

	if (currentDistance > radiusForOvershoot) {
		//		std::cout <<"Straight Flight: there's still way to go\n";
		emit sigRequestedSetValueChanged(ctrlType::HEADING_CONTROL,
				requestedHeading, true);
		//TODO Here we can add a glide angle correction to adjust for the right height at the end of straight segments;
		//This would also be kind of a P-controller
	} else {
		//		std::cout <<"Straight Flight: we are close to the target\n";
		//we are close to our target point, so let's get the correct heading for next circle
		emit sigRequestedSetValueChanged(ctrlType::HEADING_CONTROL,
				tangentialHeading, true);

		// Let's check for the end-Condition of straight flight
		// That is, when the next circle center is left or right of us, we leave straight flight
		double headingToNextCircleCenter = dc->getPosition().getHeadingCart(nextcircleCenterBlown);
		double relativeHeading = getAngularDifference(currentHeading, headingToNextCircleCenter);
//		std::cout << "relative Heading to nextCircleCenter: " <<  relativeHeading <<"\n";
		if(fabs(relativeHeading) >= 90){
			// we made it over the tangential straight line
			std::cout <<"Straight Flight: finished\n";
			isStraightFinished = true;
			dc->changeSegmentStatisticsState(false);
		}
	}
}

RunOutPhase::RunOutPhase(Position _elf, double _elfHeading,
		QObject* parent) : FlightPhase(parent) {
	elf = _elf;
	elfHeading = _elfHeading;
	segType = segmentTypeEnum::FINAL;
}

RunOutPhase::~RunOutPhase() {
}

void RunOutPhase::performFlight(void) {
	dc->changeSegmentStatisticsState(true);	//to reset;
	dc->changeSegmentStatisticsState(false);

	//nothing to do any more
	//continue straight flight and enable the controls again
	emit sigCtrlActiveStateChanged(ctrlType::RADIUS_CONTROL, false);	//just to be sure
	emit sigRequestedSetValueChanged(ctrlType::HEADING_CONTROL, elfHeading, true);
	emit sigCtrlActiveStateChanged(ctrlType::HEADING_CONTROL, true);
	emit sigCtrlActiveStateChanged(ctrlType::CLIMB_CONTROL, true);	//when doing an auto-approach, we want to have all active

	return;
}
