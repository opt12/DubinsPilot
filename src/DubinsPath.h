/*
 * DubinsPath.h
 *
 *  Created on: 20.06.2018
 *      Author: eckstein
 */

#ifndef DUBINSPATH_H_
#define DUBINSPATH_H_

#include "Position.h"
#include "enumDeclarations.h"

#include <string>

class DubinsPath {
public:
	// constructor that directly computes a path with given parameters
	DubinsPath(Position start, Position end,
			double startHeading, double endHeading,
			double circleRadius,
			double circleGlideRatio, double straightGlideRatio,
			pathTypeEnum _pathType){
		calculateDubinsPath(start, end,
			startHeading, endHeading,
			circleRadius,
			circleGlideRatio, straightGlideRatio,
			_pathType);
	}

	// function to compute a path with given parameters
	void calculateDubinsPath(Position start, Position end,
			double startHeading, double endHeading,
			double circleRadius,
			double circleGlideRatio, double straightGlideRatio,
			pathTypeEnum pathType, double windVelocity = 0.0, double windHeading = 0.0);

	std::string getPathAsMatlabCommand(void);
	std::string getExtendedPathAsMatlabCommand(void);

	double getPathLength(void) {
		return circleLength[0] + circleLength[1] + inBetweenLength
				+ finalApproachLengthExt;
	}
	double getCircleLength(void) {
		return circleLength[0] + circleLength[1];
	}
	double getStraightLength(void) {
		return inBetweenLength + finalApproachLengthExt;
	}


private:

	/* an approach in the body frame of the aircraft is given in coordinates as
	 * - the type of dubins path
	 * - the start- and the endpoint including its height data
	 * - the first circle to be flown with its center coordinate
	 * - the entry and exit angle of the first circle
	 * - the second circle to be flown with its center coordinate
	 * - the entry and exit angle of the second circle
	 * - the radius of the first and second circle (for the moment they are meant to be equal)
	 * - the glide ratio in the first and second circle and in straight flight
	 *
	 * This is supposed to be enough to define the flight path, but for convenience we cache
	 * - the entry point and the exit point of the first circle
	 * - the entry point and the exit point of the second circle
	 *
	 * All coordinates are calculated in a cartesian coordinate system, but they are stored
	 * as geodetic coordinates to be prepared for changes of the origin.
	 */

	Position startPoint, endPoint;
	Position circleCenter[2];
	Position circleCenterExt[2];
	double circleEntryAngleExt[2], circleExitAngleExt[2];
	double circleEntryAngle[2], circleExitAngle[2];
	double circleRadius = 500, circleGlideRatio = 10.4;
	double straightGlideRatio = 10.4;
	bool cachesDirty = true;
	Position circleEntry[2], circleExit[2];	//this is only for caching
	Position circleEntryExt[2], circleExitExt[2];	//this is only for caching
	double circleLength[2], inBetweenLength; //this is only for caching
	double circleLengthExt[2], inBetweenLengthExt, finalApproachLengthExt; //this is only for caching
	pathTypeEnum pathType = pathTypeEnum::LSL;	//One of LSL, RSR (LSR, RSL in the future)

	void updateCaches();
};

#endif /* DUBINSPATH_H_ */
