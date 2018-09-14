/*
 * SegmentStatistics.h
 *
 *  Created on: 30.08.2018
 *      Author: eckstein
 */

#ifndef SEGMENTSTATISTICS_H_
#define SEGMENTSTATISTICS_H_

class DataCenter;
#include "Position.h"
#include <QTime>

class SegmentStatistics {
	//statistics for flight segments (straight and circle flight)

public:
	SegmentStatistics();
	virtual ~SegmentStatistics();

	void initialize(void);

	bool isSegmentStatsActive(void) const{
		return segmentStatsActive;
	}

	void stopSegmentStatsUpdate(){
		segmentStatsActive = false;
	}

	double getCartesianPathLengthEarthFrame() const {
		return cartesianPathLengthEarthFrame;
	}

	double getCartesianPathLengthWindFrame() const {
		return cartesianPathLengthWindFrame;
	}

	double getFlightTime() const {
		return flightTime;
	}

	double getHeightLoss() const {
		return heightLoss;
	}

	const Position_Cartesian getWindDisplacement() const {
		return lastWindDisplacement-initialDisplacement;
	}

	double getTotalAngleFlown() const {
		return totalAngleFlown;
	}

	void updateSegmentData(void);

	json asJson(void) const;


	bool segmentStatsActive = false;
	Position startPosition;
	Position_Cartesian initialDisplacement;
	QTime startTime = QTime();
	Position lastPosition;
	Position_Cartesian lastWindDisplacement;
	double cartesianPathLengthEarthFrame = 0.0; //holds the Integral of the path in the xy-plane in meters
	double cartesianPathLengthWindFrame = 0.0; //holds the Integral of the path in the xy-plane in meters
	double heightLoss = 0.0;	//holds the height loss of this segment in meters
	double flightTime = 0.0;	//holds the flight time of this segment in seconds;
	double startAngle=0.0, lastAngle=0.0, totalAngleFlown = 0.0;

private:
	DataCenter* dc = NULL;
};
#endif /* SEGMENTSTATISTICS_H_ */
