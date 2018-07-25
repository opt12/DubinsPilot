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
	DubinsPath(){
		//default constructor that does nothing;
	}

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
			_pathType, false);	//TODO Parameter bool calculateShortest auf false!!!
	}

//	// constructor that directly computes a path with available parameters
//	DubinsPath(Position start, Position end,
//			double startHeading, double endHeading, pathTypeEnum _pathType){
//		calculateDubinsPath(start, end,
//			startHeading, endHeading, circleRadius,
//			circleGlideRatio, straightGlideRatio,
//			_pathType, double);
//	}

	static double calculateMinimumHeightLoss(const Position start,
			const Position end, const double startHeading, const double endHeading,
			const double _circleRadius, const double _circleGlideRatio,
			const double _straightGlideRatio, const pathTypeEnum _pathType);


	// function to compute a path with given parameters
	void calculateDubinsPath(Position start, Position end,
			double startHeading, double endHeading,
			double _circleRadius,
			double _circleGlideRatio, double _straightGlideRatio,
			pathTypeEnum _pathType, bool calculateShortest,
			double windVelocity = 0.0, double windHeading = 0.0);

//	//obsdolete without calculateShortest Parameter
//	void calculateDubinsPath(const Position start, const Position end,
//			const double startHeading, const double endHeading,
//			const double _circleRadius, const double _circleGlideRatio,
//			const double _straightGlideRatio, const pathTypeEnum _pathType,
//			const double windVelocity, const double windHeading);


	std::string getPathAsMatlabCommand(void);
//	std::string getExtendedPathAsMatlabCommand(void);

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

	json asJson();

	pathTypeEnum getPathType() const {
		return pathType;
	}

	const Position* getCircleCenter() const {
		return circleCenter;
	}

	const Position* getCircleEntry() const {
		return circleEntry;
	}

	const double* getCircleEntryAngle() const {
		return circleEntryAngle;
	}

	const Position* getCircleExit() const {
		return circleExit;
	}

	const double* getCircleExitAngle() const {
		return circleExitAngle;
	}

	double getCircleGlideRatio() const {
		return circleGlideRatio;
	}

	double getCircleRadius() const {
		return circleRadius;
	}

	const Position& getEndPoint() const {
		return endPoint;
	}

	const Position& getStartPoint() const {
		return startPoint;
	}

	double getStraightGlideRatio() const {
		return straightGlideRatio;
	}

	bool getIsValidDubinsPath() const {
		return isValidDubinsPath;
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

	bool isValidDubinsPath = false;
	double heightLoss = 0.0;
	Position startPoint, endPoint;
	Position circleCenter[2];
	double circleEntryAngle[2], circleExitAngle[2];
	double circleRotation[2], circleTotalRot=0.0;
	double circleRadius=0.0;
	double straightGlideRatio = 0.0, circleGlideRatio = 0.0;
	Position circleEntry[2], circleExit[2];
	double circleLength[2]={0, 0}, inBetweenLength=0; //this is only for caching
	double finalApproachLengthExt=0.0, finalApproachHeightLoss = 0.0;
	pathTypeEnum pathType = pathTypeEnum::LSL;	//One of LSL, RSR (LSR, RSL in the future)

	json trochoidAsJson(Position circleCenter, double circleRadius,
			double entryAngleDeg, double exitAngleDeg,
			Position_Cartesian displacement = Position_Cartesian());
};

#endif /* DUBINSPATH_H_ */
