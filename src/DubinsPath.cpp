/*
 * DubinsPath.cpp
 *
 *  Created on: 20.06.2018
 *      Author: eckstein
 */

#include <DubinsPath.h>
#include "enumDeclarations.h"
#include <math.h>
#include <vector>
#include <tuple>
#include <algorithm>    // std::transform
#include "constants.h"

/*
 * The Matlab output uses a command to draw circular segments that goes like this
 * plot_arc_cw.m:
 * function P = plot_arc(a,b,xc,yc,r)
 * % Plot a circular arc as a pie wedge.
 * % a is start of arc in radians,
 * % b is end of arc in radians,
 * % (xc,yc) is the center of the circle.
 * % r is the radius.
 * % Try this:   plot_arc(pi/4,3*pi/4,9,-4,3)
 * % Author:  Matt Fig
 * hold on
 * theta = linspace(a, b);
 * x = -r*sin(-theta) + xc;
 * y = r*cos(-theta) + yc;
 * P = plot(x, y)
 *
 * axis equal
 */
std::string DubinsPath::getPathAsMatlabCommand(void) {
	std::stringstream matlabCommand;
	Position_Cartesian tmp1 = circleCenter[0].getPosition_Cart();
	matlabCommand << "plot_arc_cw(" << to_radians(circleEntryAngle[0]) << ", "
			<< to_radians(circleExitAngle[0]) << ", " << tmp1.x << ", "
			<< tmp1.y << ", " << circleRadius << ")" << std::endl;
	Position_Cartesian tmp2 = circleCenter[1].getPosition_Cart();
	matlabCommand << "plot_arc_cw(" << to_radians(circleEntryAngle[1]) << ", "
			<< to_radians(circleExitAngle[1]) << ", " << tmp2.x << ", "
			<< tmp2.y << ", " << circleRadius << ")" << std::endl;
	tmp1 = circleExit[0].getPosition_Cart();
	tmp2 = circleEntry[1].getPosition_Cart();
	matlabCommand << "line([" << tmp1.x << " " << tmp2.x << "], [" << tmp1.y
			<< " " << tmp2.y << "])" << std::endl;
	tmp1 = circleEntry[0].getPosition_Cart();
	tmp2 = endPoint.getPosition_Cart();
	matlabCommand << "line([" << tmp1.x << " " << tmp2.x << "], [" << tmp1.y
			<< " " << tmp2.y << "])" << std::endl;

	return matlabCommand.str();
}

double DubinsPath::calculateMinimumHeightLoss(const Position start,
		const Position end, const double startHeading, const double endHeading,
		const double _circleRadius, const double _circleGlideRatio,
		const double _straightGlideRatio, const pathTypeEnum _pathType) {
	// TODO hier wird Code wiederholt, der auch in der echten Pfadberechnung
	// gebraucht wird. Das stinkt ein bisschen.

	// get the cartesian coordinates of the start, and endpoints
	// firstly, the endpoint is used as the origin
	Position_Cartesian startCart = end.getCartesianDifference(start), endCart =
			Position_Cartesian(0, 0, end.getPosition_WGS84().height);

	// rotate the system to have the endHeading in 270° direction (-X-axis)
	double rotation = fmod(270.0 - endHeading, 360);

	// all variables with suffix _t are transformed with this rotation
	double startHead_t = fmod(startHeading + rotation, 360), endHead_t = fmod(
			endHeading + rotation, 360);
	Position_Cartesian start_t, end_t;
	rotatePoint(startCart.x, startCart.y, rotation, start_t.x, start_t.y);
	start_t.z = startCart.z;
	rotatePoint(endCart.x, endCart.y, rotation, end_t.x, end_t.y);
	end_t.z = endCart.z;

	//start with the calculation of the shortest dubins path:
	Position_Cartesian circleCenter_t[2];
	double deltaX, deltaY;	// cartesian distance between the two circle centers
	double headingStraight_t;	// heading of the interconnection tangential
	double rotCircle[2];	// turning angle within each circle
	double circleLength[2]; //the length of each circle segment
	double heightLossCircle[2], heightLossStraight;
	double straightLength;

	int dirSign = (_pathType == +pathTypeEnum::LSL) ? -1 : 1; // this is used to use the correct direction of turns for LSL and RSR

	// der erste Kreismittelpunkt kann direkt berechnet werden
	circleCenter_t[0].x = start_t.x
			- _circleRadius * sin(to_radians(-dirSign * 90 - startHead_t));
	circleCenter_t[0].y = start_t.y
			+ _circleRadius * cos(to_radians(-dirSign * 90 - startHead_t));
	// der Mittelpunkt des zweiten Kreises kann auch direkt berechnet werden
	// der zweite Kreis hat seinen Mittelpunkt einen Radius unterhalb und rechts des Zielpunkts
	circleCenter_t[1].y = end_t.y + _circleRadius * (dirSign);// cos(to_radians(90 - endHead_t)) == 1, da endHead_t == 270;
	circleCenter_t[1].x = end_t.x - _circleRadius * (0.0); //sin(to_radians(90 - endHead_t)) == 0, da endHead_t == 270;

	// der Abstand in y-Richtung
	deltaY = circleCenter_t[1].y - circleCenter_t[0].y;
	deltaX = circleCenter_t[1].x - circleCenter_t[0].x;
	// the direction between the center points of the circles
	// fmod(..., -360) to get the left turning circle
	headingStraight_t = fmod(to_degrees(atan2(deltaX, deltaY)),
			dirSign * 360.0);
	// die Rotationen in den einzelnen Kreisen
	rotCircle[0] = fmod(headingStraight_t - startHead_t + dirSign * 2 * 360,
			dirSign * 360.0);
	rotCircle[1] = fmod(endHead_t - headingStraight_t + dirSign * 2 * 360,
			dirSign * 360.0);

	//damit kann der Höhenverlust des kürzesten Weges ausgerechnet werden
	circleLength[0] = fabs(_circleRadius * to_radians(rotCircle[0]));
	heightLossCircle[0] = -circleLength[0] / _circleGlideRatio;
	circleLength[1] = fabs(_circleRadius * to_radians(rotCircle[1]));
	heightLossCircle[1] = -circleLength[1] / _circleGlideRatio;
	straightLength = sqrt(deltaX * deltaX + deltaY * deltaY);
	heightLossStraight = -straightLength / _straightGlideRatio;

	// minimumHeightLoss is given as positive value,
	// so it must be subtracted from the startpoint height
	return (heightLossCircle[0] + heightLossCircle[1] + heightLossStraight);
}

void DubinsPath::calculateDubinsPath(const Position start,
		const Position end, const double startHeading, const double endHeading,
		const double _circleRadius, const double _circleGlideRatio,
		const double _straightGlideRatio, const pathTypeEnum _pathType,
		bool calculateShortest, const double windVelocity, const double windHeading) {

	Position_Cartesian circleCenter_t[2], circleEntry_t[2], circleExit_t[2];
	double requiredHeightDiff;

	pathType = _pathType;
	circleRadius = _circleRadius;
	circleGlideRatio = _circleGlideRatio;
	straightGlideRatio = _straightGlideRatio;
	isValidDubinsPath = true;//we are optimistic and reset this assumption in case it's untrue;

	// get the cartesian coordinates of the start, and endpoints
	// firstly, the endpoint is used as the origin
	Position_Cartesian startCart = end.getCartesianDifference(start);
	startCart.z = start.getPosition_WGS84().height;	//now we have the real elevation in here
	Position_Cartesian endCart = Position_Cartesian(0, 0, end.getPosition_WGS84().height);

	requiredHeightDiff = startCart.z- endCart.z;

	//start with the calculation of the shortest dubins path:


	// rotate the system to have the endHeading in 270° direction (-X-axis)
	double rotation = fmod(270.0 - endHeading, 360);

	// all variables with suffix _t are transformed with this rotation
	double startHead_t = fmod(startHeading + rotation, 360), endHead_t = fmod(
			endHeading + rotation, 360);
	Position_Cartesian start_t, end_t;
	rotatePoint(startCart.x, startCart.y, rotation, start_t.x, start_t.y);
	start_t.z = startCart.z;
	rotatePoint(endCart.x, endCart.y, rotation, end_t.x, end_t.y);
	end_t.z = endCart.z;

	//start with the calculation of the shortest dubins path:
	double deltaX, deltaY;// cartesian distance between the two circle centers
	double headingStraight_t;// heading of the interconnection tangential
	double rotCircle[2];// turning angle within each circle
	double circleLength[2];//the length of each circle segment
	double heightLossCircle[2], heightLossStraight;
	double straightLength;

	int dirSign = (_pathType == +pathTypeEnum::LSL) ? -1 : 1;// this is used to use the correct direction of turns for LSL and RSR

	// der erste Kreismittelpunkt kann direkt berechnet werden
	circleCenter_t[0].x = start_t.x
	- _circleRadius * sin(to_radians(-dirSign * 90 - startHead_t));
	circleCenter_t[0].y = start_t.y
	+ _circleRadius * cos(to_radians(-dirSign * 90 - startHead_t));
	// der Mittelpunkt des zweiten Kreises kann auch direkt berechnet werden
	// der zweite Kreis hat seinen Mittelpunkt einen Radius unterhalb und rechts des Zielpunkts
	circleCenter_t[1].y = end_t.y + _circleRadius * (dirSign);// cos(to_radians(90 - endHead_t)) == 1, da endHead_t == 270;
	circleCenter_t[1].x = end_t.x - _circleRadius * (0.0);//sin(to_radians(90 - endHead_t)) == 0, da endHead_t == 270;

	// der Abstand in y-Richtung
	deltaY = circleCenter_t[1].y - circleCenter_t[0].y;
	deltaX = circleCenter_t[1].x - circleCenter_t[0].x;
	// the direction between the center points of the circles
	// fmod(..., -360) to get the left turning circle
	headingStraight_t = fmod(to_degrees(atan2(deltaX, deltaY)),
			dirSign * 360.0);
	// die Rotationen in den einzelnen Kreisen
	rotCircle[0] = fmod(headingStraight_t - startHead_t + dirSign * 2 * 360,
			dirSign * 360.0);
	rotCircle[1] = fmod(endHead_t - headingStraight_t + dirSign * 2 * 360,
			dirSign * 360.0);
	circleRotation[0] = rotCircle[0];
	circleRotation[1] = rotCircle[1];
	double rotTotal = rotCircle[0] + rotCircle[1];	//for caching
	circleTotalRot = circleRotation[0] + circleRotation[1];

	circleEntry_t[0] = start_t;
	circleExit_t[0].x = circleCenter_t[0].x
			+ circleRadius * cos(to_radians(-headingStraight_t));
	circleExit_t[0].y = circleCenter_t[0].y
			+ circleRadius * sin(to_radians(-headingStraight_t));
	circleEntry_t[1].x = circleCenter_t[1].x
			+ circleRadius * cos(to_radians(-headingStraight_t));
	circleEntry_t[1].y = circleCenter_t[1].y
			+ circleRadius * sin(to_radians(-headingStraight_t));
	circleExit_t[1] = end_t;
	circleEntryAngle[0] = startHeading + (-dirSign *90);
	circleExitAngle[0] = startHeading + circleRotation[0] + (-dirSign *90);
	circleEntryAngle[1] = endHeading - circleRotation[1] + (-dirSign *90);
	circleExitAngle[1] = endHeading + (-dirSign *90);

	circleLength[0] = fabs(_circleRadius * to_radians(circleRotation[0]));
	heightLossCircle[0] = -circleLength[0] / circleGlideRatio;
	circleLength[1] = fabs(_circleRadius * to_radians(circleRotation[1]));
	heightLossCircle[1] = -circleLength[1] / circleGlideRatio;
	straightLength = sqrt(deltaX * deltaX + deltaY * deltaY);
	heightLossStraight = -straightLength / straightGlideRatio;

	// setze die Höhendaten für die einzelnen Kreispunkte
	circleEntry_t[0].z = startCart.z;
	circleExit_t[0].z = startCart.z - heightLossCircle[0];
	circleEntry_t[1].z = circleExit_t[0].z - heightLossStraight;
	circleExit_t[1].z = circleEntry_t[1].z - heightLossCircle[1];

	//now unrotate the coordinate system
	Position_Cartesian temp;
	rotatePoint(circleEntry_t[0].x, circleEntry_t[0].y, -rotation, temp.x,
			temp.y);
	temp.z = circleEntry_t[0].z - end.getHeight();
	circleEntry[0] = Position(end, temp);
	rotatePoint(circleExit_t[0].x, circleExit_t[0].y, -rotation, temp.x,
			temp.y);
	temp.z = circleExit_t[0].z - end.getHeight();
	circleExit[0] = Position(end, temp);
	rotatePoint(circleEntry_t[1].x, circleEntry_t[1].y, -rotation, temp.x,
			temp.y);
	temp.z = circleEntry_t[1].z - end.getHeight();
	circleEntry[1] = Position(end, temp);
	rotatePoint(circleExit_t[1].x, circleExit_t[1].y, -rotation, temp.x,
			temp.y);
	temp.z = circleExit_t[1].z - end.getHeight();
	circleExit[1] = Position(end, temp);

	rotatePoint(circleCenter_t[0].x, circleCenter_t[0].y, -rotation, temp.x,
			temp.y);
	temp.z = (circleEntry_t[0].z + circleExit_t[0].z)/2.0 - end.getHeight();
	circleCenter[0] = Position(end, temp);
	rotatePoint(circleCenter_t[1].x, circleCenter_t[1].y, -rotation, temp.x,
			temp.y);
	temp.z = (circleEntry_t[1].z + circleExit_t[1].z)/2.0 - end.getHeight();
	circleCenter[1] = Position(end, temp);

	//set everything else in the DubinsPath object
	startPoint = start;
	endPoint = end;

	double overallCircleHeightLoss = heightLossCircle[0] + heightLossCircle[1];

	//damit kann der Höhenverlust des kürzesten Weges ausgerechnet werden
	heightLoss = heightLossStraight + overallCircleHeightLoss;

	if (heightLoss >= requiredHeightDiff + EPS) {
		isValidDubinsPath = false;	// we won't make it there;
		std::cout
				<< "ATTENTION!!! (1) The endpoint is not reachable due to height constraints.\n";
		return;
	}

	if(calculateShortest){
		//mehr brauchen wir nicht
		return;
	}

	//implicit else für erweiterte Pfadberechnung


	//berechne die Länge des final approach:
	//wenn das zu wenig ist, dann muss der final approach verlängert werden
	double requiredHeightLossStraight = requiredHeightDiff - overallCircleHeightLoss;	// Sollwert

	if(fabs(requiredHeightLossStraight-heightLossStraight) < EPS){
		std::cout
				<< "Shortest Path matches the extended Path due to height constraints.\n";
		std::cout<< "requiredHeightDiff: " << requiredHeightDiff;
		std::cout<< "; Achieved: heightLoss: " << heightLoss<< ";\n";
		std::cout<< "Flown Angle[0]: "<< circleRotation[0]<<"; Flown Angle[1]: "<< circleRotation[1]<<";\n";
		std::cout<< "total flown Angle: "<< circleRotation[0]+ circleRotation[1]<<";\n";
		return;
	}

	// jetzt muss der final Approach verlängert werden
	double requiredStraightLength = -(straightGlideRatio * requiredHeightLossStraight); //Sollwert

	finalApproachLengthExt = -(deltaY * deltaY + deltaX * deltaX
			- requiredStraightLength * requiredStraightLength)
			/ (2 * (requiredStraightLength + deltaX));

	finalApproachHeightLoss = (-finalApproachLengthExt / straightGlideRatio);

	// verschiebe den Zielpunkt um Länge finalApproachLengthExt in Richtung -endHeading
	// inklusive Höhenkorrektur
	Position_Cartesian endPointCorrection;
	rotatePoint(0.0, finalApproachLengthExt, -(180-endHeading), endPointCorrection.x, endPointCorrection.y);
	Position correctedEndPoint = Position(end, endPointCorrection);
	correctedEndPoint.setHeight(end.getHeight() + finalApproachHeightLoss);

	// starte eine neue kürzeste Dubins Berechnung zum neuen Zielpunkt
	// rekursiver Aufruf dieses Mal mit calculateShortest = true
	calculateDubinsPath(start, correctedEndPoint, startHeading, endHeading, circleRadius,
			circleGlideRatio, straightGlideRatio, pathType,
			true, // shortest path to correctedEndPoint
			windVelocity, windHeading);

	// prüfe ob isDubinsPathFeasible
	if(isValidDubinsPath && (fabs(circleTotalRot - rotTotal) < EPS)){
		// we got the path we were looking for
		heightLoss+= finalApproachHeightLoss;
		endPoint = end;

		std::cout<< "Found feasible Dubins Path:\n";
		std::cout<< "requiredHeightDiff: " << requiredHeightDiff
				<< "; Achieved: heightLoss: " << heightLoss<< ";\n";
		std::cout<< "Flown Angle[0]: "<< circleRotation[0]<<"; Flown Angle[1]: "<< circleRotation[1]<<";\n";
		std::cout<< "total flown Angle: "<< circleTotalRot<<";\n";
		return;
	}

	//implicit else: the longer Dubins path is not feasible
	// this must be due to angle changes in flying
	std::cout << "adapted Dubins Path has angle of " << circleTotalRot<<
			"; shortest Dubins Path has angle of "<< rotTotal<< ";\n";

	double fullCircleHeight = 2 * M_PI * circleRadius / (-circleGlideRatio);


	if(!isValidDubinsPath && (fabs(circleTotalRot - rotTotal) > 350)){
		// The new path was not feasible as it required one more circle so we try to
		// shorten the final approach by the height of a full cirlce
		// jetzt muss der final Approach verlängert werden
		requiredHeightLossStraight = requiredHeightDiff - overallCircleHeightLoss - fullCircleHeight;	// Sollwert
		requiredStraightLength = -(straightGlideRatio * requiredHeightLossStraight); //Sollwert

		finalApproachLengthExt = -(deltaY * deltaY + deltaX * deltaX
				- requiredStraightLength * requiredStraightLength)
				/ (2 * (requiredStraightLength + deltaX));

		finalApproachHeightLoss = (-finalApproachLengthExt / straightGlideRatio);

		// verschiebe den Zielpunkt um Länge finalApproachLengthExt in Richtung -endHeading
		// inklusive Höhenkorrektur
		Position_Cartesian endPointCorrection;
		rotatePoint(0.0, finalApproachLengthExt, -(180-endHeading), endPointCorrection.x, endPointCorrection.y);
		Position correctedEndPoint = Position(end, endPointCorrection);
		correctedEndPoint.setHeight(end.getHeight() + finalApproachHeightLoss);

		// starte eine neue kürzeste Dubins Berechnung zum neuen Zielpunkt
		// rekursiver Aufruf dieses Mal mit calculateShortest = true
		calculateDubinsPath(start, correctedEndPoint, startHeading, endHeading, circleRadius,
				circleGlideRatio, straightGlideRatio, pathType,
				true, // shortest path to correctedEndPoint
				windVelocity, windHeading);

		// prüfe ob isDubinsPathFeasible
		if(isValidDubinsPath){
			if(fabs(requiredHeightDiff - heightLoss - finalApproachHeightLoss) < EPS){
				// we got the path we were looking for
				heightLoss+= finalApproachHeightLoss;
				endPoint = end;

				std::cout<< "Found feasible Dubins Path:\n";
				std::cout<< "requiredHeightDiff: " << requiredHeightDiff
						<< "; Achieved: heightLoss: " << heightLoss<< ";\n";
				std::cout<< "Flown Angle[0]: "<< circleRotation[0]<<"; Flown Angle[1]: "<< circleRotation[1]<<";\n";
				std::cout<< "total flown Angle: "<< circleTotalRot<<";\n";
				return;
			} else {
				std::cout << "ATTENTION!!! (2) The endpoint is not reachable due to height constraints.\n";
				std::cout << "No Additional Circle will be flown: \n";
				std::cout << "adapted Dubins Path has angle of " << circleTotalRot<<
						"; shortest Dubins Path has angle of "<< rotTotal<< ";\n";
				isValidDubinsPath = false;
				return;
			}
		}
	}


	if (fabs(requiredHeightDiff - fullCircleHeight- (heightLoss+ finalApproachHeightLoss)) < EPS) {
		// wir müssen einen Kreis mehr fliegen, weil die Verlängerung die Winkel ändert
		std::cout <<"INSERTING another entry circle round to achieve the required height loss.\n";
		circleExitAngle[0] = circleExitAngle[0] + dirSign*360;
		circleRotation[0] = circleRotation[0] + dirSign*360;
		circleExit[0].setHeight(circleExit[0].getHeight()- fullCircleHeight);
		circleCenter[0].setHeight(circleCenter[0].getHeight()- fullCircleHeight/2);
		circleEntry[1].setHeight(circleEntry[1].getHeight()- fullCircleHeight);
		circleExit[1].setHeight(circleExit[1].getHeight()- fullCircleHeight);
		circleCenter[1].setHeight(circleCenter[1].getHeight()- fullCircleHeight);
		endPoint.setHeight(endPoint.getHeight()- fullCircleHeight);
		heightLoss+= finalApproachHeightLoss + fullCircleHeight;
		endPoint = end;

		std::cout<< "Found feasible Dubins Path with additional full entry circle: requiredHeightDiff: " << requiredHeightDiff;
		std::cout<< "; Achieved: heightLoss: " << heightLoss<< ";\n";
		std::cout<< "Flown Angle[0]: "<< circleRotation[0]<<"; Flown Angle[1]: "<< circleRotation[1]<<";\n";
		std::cout<< "total flown Angle: "<< circleRotation[0]+ circleRotation[1]<<";\n";
		return;
	}

	//implicit else; Nein, es geht nicht
	isValidDubinsPath = false;
	std::cout <<"ATTENTION (5): we should never get here\n";
	isValidDubinsPath = false;
}


json DubinsPath::asJson() {
	json j;
	if (!isValidDubinsPath) {
		return j;
	}

	j["pathType"] = pathType._to_string();
	j["startingPoint"]= {startPoint.getPosition_WGS84().lati, startPoint.getPosition_WGS84().longi};
	j["endPoint"]= {endPoint.getPosition_WGS84().lati, endPoint.getPosition_WGS84().longi};
	j["trochoidOne"] = trochoidAsJson(circleCenter[0], circleRadius,
			circleEntryAngle[0], circleExitAngle[0]);
	j["trochoidTwo"] = trochoidAsJson(circleCenter[1], circleRadius,
			circleEntryAngle[1], circleExitAngle[1]);
	j["startHeading"] = circleEntryAngle[0]
			+ ((pathType == +pathTypeEnum::LSL) ? -90 : +90);
	j["endHeading"] = circleExitAngle[1]
			+ ((pathType == +pathTypeEnum::LSL) ? -90 : +90);
	j["isValidDubinsPath"] = isValidDubinsPath;
	return j;
}

json DubinsPath::trochoidAsJson(Position circleCenter, double circleRadius,
		double entryAngleDeg, double exitAngleDeg,
		Position_Cartesian displacement) {
	const int DEGREE_SCALE = 10;
	json j;
	j["trochoRadius"] = circleRadius;
	j["trochoCenterStart"] = {circleCenter.getPosition_WGS84().lati, circleCenter.getPosition_WGS84().longi};
	j["trochoCenterEnd"] = {
		(circleCenter+displacement).getPosition_WGS84().lati,
		(circleCenter+displacement).getPosition_WGS84().longi
	};

	int steps = abs((exitAngleDeg - entryAngleDeg) / DEGREE_SCALE) + 1;	//one point each ~5°
	double stepSize = (exitAngleDeg - entryAngleDeg) / steps;// hw many degrees exactly, including sign
	std::vector<Position> points = std::vector<Position>(steps + 1);
	for (int i = 0; i <= steps; i++) {
		Position_Cartesian circleRay = Position_Cartesian(0.0, circleRadius,
				0.0).rotate(entryAngleDeg + i * stepSize);
		points[i] = Position(circleCenter + (i * displacement / steps),
				circleRay);	//TODO muss hier durch (steps+1) geteilt werden?
	}
	std::vector<std::tuple<double, double>> latlongArray;
	std::transform(points.begin(), points.end(),
			std::back_inserter(latlongArray), [](Position pos) {
				std::tuple<double, double> tup {
					pos.getPosition_WGS84().lati,
					pos.getPosition_WGS84().longi
				};
				return tup;
			});
	j["trochoBorder"] = latlongArray;
	return j;
}

