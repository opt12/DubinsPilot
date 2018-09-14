/*
 * SegmentStatistics.cpp
 *
 *  Created on: 30.08.2018
 *      Author: eckstein
 */

#include <SegmentStatistics.h>
#include "utils.h"
#include "DataCenter.h"

#include <stdio.h>


SegmentStatistics::~SegmentStatistics() {
	// TODO Auto-generated destructor stub
}

SegmentStatistics::SegmentStatistics(){
	//The dc pointer to the DataCenter is only set in initialize
}

void SegmentStatistics::initialize(){

	dc = DataCenter::getInstance();

	segmentStatsActive = true;

	startPosition= dc->getPosition();;
	initialDisplacement= dc->getWindDisplacement();;
	startTime = QTime::currentTime();
	lastPosition = startPosition;
	lastWindDisplacement= initialDisplacement;;
	cartesianPathLengthEarthFrame = 0.0;
	cartesianPathLengthWindFrame = 0.0;
	heightLoss = 0.0;
	flightTime = 0.0;
	startAngle= dc->getTrue_psi();
	lastAngle=startAngle;
	totalAngleFlown = 0.0;
}

void SegmentStatistics::updateSegmentData(void){
	if(!segmentStatsActive){
		return;	//no need to update anything, just freeze it;
	}
	Position currentPosition = dc->getPosition();
	Position_Cartesian travel = lastPosition.getCartesianDifference(currentPosition);
	double distance = travel.length();

	Position_Cartesian currentWindDisplacement= dc->getWindDisplacement();
	Position_Cartesian windCorrection = lastWindDisplacement - currentWindDisplacement ;

	double distanceWithoutWind = (travel + windCorrection).length();

	cartesianPathLengthEarthFrame += distance;
	cartesianPathLengthWindFrame += distanceWithoutWind;
	heightLoss = startPosition.getAltitudeDiff(currentPosition);
	lastPosition = currentPosition;
	lastWindDisplacement = currentWindDisplacement;

//	std::cout <<"travel: "<< travel.asJson() <<"\n";
//	std::cout <<"distance: "<< distance <<"\tLengthEarthFrame: "<< cartesianPathLengthEarthFrame <<"\n";
//
//	std::cout <<"windCorrection: "<< windCorrection.asJson() <<"\n";
//	std::cout <<"distanceWithoutWind: "<< distanceWithoutWind <<
//			"\tLengthWindFrame: "<< cartesianPathLengthWindFrame <<"\n";

	double currentAngle = dc->getTrue_psi();
	totalAngleFlown += getAngularDifference(lastAngle, currentAngle);
	lastAngle = currentAngle;

	flightTime = startTime.msecsTo(QTime::currentTime())/1000.0;
}

json SegmentStatistics::asJson(void) const {
	json j;

	j["flightTime"] = flightTime;
	j["cartesianPathLengthEarthFrame"] = cartesianPathLengthEarthFrame;
	j["cartesianPathLengthWindFrame"] = cartesianPathLengthWindFrame;
	j["heightLoss"] = heightLoss;
	j["totalWindDisplacement.x"] = (lastWindDisplacement-initialDisplacement).x;
	j["totalWindDisplacement.y"] = (lastWindDisplacement-initialDisplacement).y;
	j["glideAngleEarthFrame"] =  to_degrees(atan(heightLoss/cartesianPathLengthEarthFrame));
	j["glideAngleWindFrame"] =  to_degrees(atan(heightLoss/cartesianPathLengthWindFrame));
	j["totalAngleFlown"] = totalAngleFlown;


//	std::cout << "Flight Segment:\t"<< j.dump(4) << std::endl;
	return j;
}


