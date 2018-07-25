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

//std::string DubinsPath::getExtendedPathAsMatlabCommand(void) {
//	std::stringstream matlabCommand;
//	Position_Cartesian tmp1 = circleCenterExt[0].getPosition_Cart();
//	matlabCommand << "plot_arc_cw(" << to_radians(circleEntryAngleExt[0])
//			<< ", " << to_radians(circleExitAngleExt[0]) << ", " << tmp1.x
//			<< ", " << tmp1.y << ", " << circleRadius << ")" << std::endl;
//	Position_Cartesian tmp2 = circleCenterExt[1].getPosition_Cart();
//	matlabCommand << "plot_arc_cw(" << to_radians(circleEntryAngleExt[1])
//			<< ", " << to_radians(circleExitAngleExt[1]) << ", " << tmp2.x
//			<< ", " << tmp2.y << ", " << circleRadius << ")" << std::endl;
//	tmp1 = circleExitExt[0].getPosition_Cart();
//	tmp2 = circleEntryExt[1].getPosition_Cart();
//	matlabCommand << "line([" << tmp1.x << " " << tmp2.x << "], [" << tmp1.y
//			<< " " << tmp2.y << "])" << std::endl;
//	tmp1 = circleExitExt[1].getPosition_Cart();
//	tmp2 = endPoint.getPosition_Cart();
//	matlabCommand << "line([" << tmp1.x << " " << tmp2.x << "], [" << tmp1.y
//			<< " " << tmp2.y << "])" << std::endl;
//	tmp1 = circleEntryExt[0].getPosition_Cart();
////	tmp2 = endPoint.getPosition_Cart();
//	matlabCommand << "line([" << tmp1.x << " " << tmp2.x << "], [" << tmp1.y
//			<< " " << tmp2.y << "])" << std::endl;
//
//	return matlabCommand.str();
//}

//void DubinsPath::updateCaches(void) {
//	//updates all the cached values whenever a value of circle one is changed
//	// coordinates around a center with given angles is given as
//	// x = -r*sin(-theta) + xc;
//	// y = r*cos(-theta) + yc;
//	Position_Cartesian tmp1 = circleCenter[0].getPosition_Cart();
//	circleEntry[0] = Position(Position::getOrigin_WGS84(),
//			-circleRadius * sin(-circleEntryAngle[0]) + tmp1.x,
//			-circleRadius * cos(-circleEntryAngle[0]) + tmp1.y, tmp1.z);//TODO this is not really correct
//	circleExit[0] = Position(Position::getOrigin_WGS84(),
//			-circleRadius * sin(-circleExitAngle[0]) + tmp1.x,
//			-circleRadius * cos(-circleExitAngle[0]) + tmp1.y, tmp1.z);	//TODO this is not really correct
//
//	Position_Cartesian tmp2 = circleCenter[1].getPosition_Cart();
//	circleEntry[1] = Position(Position::getOrigin_WGS84(),
//			-circleRadius * sin(-circleEntryAngle[1]) + tmp2.x,
//			-circleRadius * cos(-circleEntryAngle[1]) + tmp2.y, tmp2.z);//TODO this is not really correct
//
//	circleExit[1] = Position(Position::getOrigin_WGS84(),
//			-circleRadius * sin(-circleExitAngle[1]) + tmp2.x,
//			-circleRadius * cos(-circleExitAngle[1]) + tmp2.y, tmp2.z);	//TODO this is not really correct
//
//	circleLength[0] = circleRadius * (circleExitAngle[0] - circleEntryAngle[0])
//			* 2 * M_PI;
//	circleLength[1] = circleRadius * (circleExitAngle[1] - circleEntryAngle[1])
//			* 2 * M_PI;
//	inBetweenLength = circleExit[0].getDistanceCart(circleEntry[1]);
//	finalApproachLengthExt = circleExit[1].getDistanceCart(endPoint);
//	cachesDirty = false;
//}

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
	circleExitAngle[0] = startHeading + rotCircle[0] + (-dirSign *90);
	circleEntryAngle[1] = endHeading - rotCircle[1] + (-dirSign *90);
	circleExitAngle[1] = endHeading + (-dirSign *90);

	circleLength[0] = fabs(_circleRadius * to_radians(rotCircle[0]));
	heightLossCircle[0] = -circleLength[0] / circleGlideRatio;
	circleLength[1] = fabs(_circleRadius * to_radians(rotCircle[1]));
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

	//damit kann der Höhenverlust des kürzesten Weges ausgerechnet werden
	heightLoss = heightLossStraight + heightLossCircle[0]
			+ heightLossCircle[1];

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
	double overallCircleHeightLoss = heightLossCircle[0] + heightLossCircle[1];
	//wenn das zu wenig ist, dann muss der final approach verlängert werden
	double requiredHeightLossStraight = requiredHeightDiff - overallCircleHeightLoss;	// Sollwert
	if (heightLossStraight > requiredHeightLossStraight + EPS) {
		isValidDubinsPath = false;
		// TODO Error Handling für inkonsistente Höhenangaben
		std::cout
				<< "ATTENTION!!! (2) The endpoint is not reachable due to height constraints.\n";
		return;
	}

	if(fabs(requiredHeightLossStraight-heightLossStraight) < EPS){
		std::cout
				<< "Shortest Path matches the extended Path due to height constraints.\n";
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
	rotatePoint(0.0, finalApproachLengthExt, -endHeading, endPointCorrection.x, endPointCorrection.y);
	Position correctedEndPoint = Position(end, endPointCorrection);
	correctedEndPoint.setHeight(end.getPosition_WGS84().height + finalApproachHeightLoss);

	// starte eine neue kürzeste Dubins Berechnung zum neuen Zielpunkt
	// rekursiver Aufruf dieses Mal mit calculateShortest = true
	calculateDubinsPath(start, correctedEndPoint, startHeading, endHeading, circleRadius,
			circleGlideRatio, straightGlideRatio, pathType,
			true, // shortest path to correctedEndPoint
			windVelocity, windHeading);

	// prüfe ob isDubinsPathFeasible
	if(!isValidDubinsPath){
		std::cout <<"The extended dubins Path is not directly feasible. Checking different angles flown...\n";
	}

	// Prüfe ob Höhenbedingungen noch erfüllt sind
	if(fabs(requiredHeightDiff - (heightLoss+ finalApproachHeightLoss)) < EPS){
		// alles ist prima, der verlängerte Pfad funktioniert
		// korrigiere endPoint und HeightLoss
		heightLoss+= finalApproachHeightLoss;
		endPoint = end;

		std::cout<< "Found feasible Dubins Path: requiredHeightDiff: " << requiredHeightDiff;
		std::cout<< "; Achieved: heightLoss: " << heightLoss<< ";\n";
		std::cout<< "Flown Angle[0]: "<< circleRotation[0]<<"; Flown Angle[1]: "<< circleRotation[1]<<";\n";
		std::cout<< "total flown Angle: "<< circleRotation[0]+ circleRotation[1]<<";\n";
		return;
	}

	// implicit else
	double fullCircleHeight = 2 * M_PI * circleRadius / (-circleGlideRatio);

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

	//implicit else
	// wir müssen ein Kreishöhe aus dem Final Approach rausnehmen weil der Pfad jetzt einen Kreis mehr braucht
	// dann müssen wir nochmal rechnen

	if(requiredHeightDiff - overallCircleHeightLoss - fullCircleHeight >= heightLossStraight-EPS){
		// es gibt eine Chance, einen Kreis mehr zu fliegen und trotzdem die Höhe noch zu schaffen
		requiredHeightLossStraight = requiredHeightDiff - overallCircleHeightLoss - fullCircleHeight;	// Sollwert
		requiredStraightLength = -(straightGlideRatio * requiredHeightLossStraight); //Sollwert

		finalApproachLengthExt = -(deltaY * deltaY + deltaX * deltaX
				- requiredStraightLength * requiredStraightLength)
				/ (2 * (requiredStraightLength + deltaX));

		if(finalApproachLengthExt < 0.0){
			isValidDubinsPath = false;
			std::cout <<"ATTENTION (6): The extended Dubins Path is not feasible\n";
			std::cout <<"requiredHeightDiff: "<< requiredHeightDiff<<";\t feasibleHeightLoss: "
					<< heightLoss+ finalApproachHeightLoss<<";\n";
			return;
		}

		finalApproachHeightLoss = (-finalApproachLengthExt / straightGlideRatio);
		endPointCorrection.z = 0;;
		rotatePoint(0.0, finalApproachLengthExt, -endHeading, endPointCorrection.x, endPointCorrection.y);
		correctedEndPoint = Position(end, endPointCorrection);
		correctedEndPoint.setHeight(end.getPosition_WGS84().height + finalApproachHeightLoss);

		// starte eine neue kürzeste Dubins Berechnung zum neuen Zielpunkt
		// rekursiver Aufruf dieses Mal mit calculateShortest = true
		calculateDubinsPath(start, correctedEndPoint, startHeading, endHeading, circleRadius,
				circleGlideRatio, straightGlideRatio, pathType,
				true, // shortest path to correctedEndPoint
				windVelocity, windHeading);

		// prüfe ob isDubinsPathFeasible
		if(!isValidDubinsPath){
			std::cout <<"ATTENTION (4): The extended Dubins Path is not feasible\n";
			std::cout <<"requiredHeightDiff: "<< requiredHeightDiff<<";\t feasibleHeightLoss: "
					<< heightLoss+ finalApproachHeightLoss<<";\n";
			return;
		}
		// korrigiere endPoint und HeightLoss
		heightLoss+= finalApproachHeightLoss;
		endPoint = end;
		//prüfe ob wirklich ein weiterer Kreis geflogen wurde
		if(fabs(heightLoss - requiredHeightDiff)> EPS){
			isValidDubinsPath = false;
			std::cout <<"ATTENTION (7): The extended Dubins Path is not feasible\n";
			std::cout <<"requiredHeightDiff: "<< requiredHeightDiff<<";\t feasibleHeightLoss: "
					<< heightLoss<<";\n";
			return;
		}
		std::cout<< "Found feasible Dubins Path with changed circle angles: requiredHeightDiff: " << requiredHeightDiff;
		std::cout<< "; Achieved: heightLoss: " << heightLoss<< ";\n";
		std::cout<< "Flown Angle[0]: "<< circleRotation[0]<<"; Flown Angle[1]: "<< circleRotation[1]<<";";
		std::cout<< "total flown Angle: "<< circleRotation[0]+ circleRotation[1]<<";\n";
		return;
	}


	//implicit else; Nein, es geht nicht
	std::cout <<"ATTENTION (5): we should never get here\n";
	isValidDubinsPath = false;
}

//obsdolete without calculateShortest Parameter
//void DubinsPath::calculateDubinsPath(const Position start, const Position end,
//		const double startHeading, const double endHeading,
//		const double _circleRadius, const double _circleGlideRatio,
//		const double _straightGlideRatio, const pathTypeEnum _pathType,
//		const double windVelocity, const double windHeading) {
//
//	pathType = _pathType;
//	circleRadius = _circleRadius;
//	circleGlideRatio = _circleGlideRatio;
//	straightGlideRatio = _straightGlideRatio;
//	isValidDubinsPath = true;//we are optimistic and reset this assumption in case it's untrue;
//
//	// get the cartesian coordinates of the start, and endpoints
//	// firstly, the endpoint is used as the origin
//	Position_Cartesian startCart = end.getCartesianDifference(start);
//	startCart.z += end.getPosition_WGS84().height;//now we have the real elevation in here
//	Position_Cartesian endCart = Position_Cartesian(0, 0,
//			end.getPosition_WGS84().height);
//
//	double requiredHeightDiff = startCart.z - endCart.z;
//
//	// rotate the system to have the endHeading in 270° direction (-X-axis)
//	double rotation = 270.0 - endHeading;
//
//	// all variables with suffix _t are transformed with this rotation
//	double startHead_t = fmod(startHeading + rotation, 360), endHead_t = fmod(
//			endHeading + rotation, 360);
//	Position_Cartesian start_t, end_t;
//	rotatePoint(startCart.x, startCart.y, rotation, start_t.x, start_t.y);
//	start_t.z = startCart.z;
//	rotatePoint(endCart.x, endCart.y, rotation, end_t.x, end_t.y);
//	end_t.z = endCart.z;
//
//	//start with the calculation of the shortest dubins path:
//	Position_Cartesian circleCenter_t[2], circleEntry_t[2], circleExit_t[2];
//	Position_Cartesian circleCenterExt_t[2], circleEntryExt_t[2],
//			circleExitExt_t[2];
//	double deltaX, deltaY;	// cartesian distance between the two circle centers
//	double headingStraight_t;	// heading of the interconnection tangential
//	double rotCircle[2];	// turning angle within each circle
//	double rotCircleExt[2];	// turning angle within each circle of the extended dubins Path
//	double heightLossCircle[2], heightLossStraight;
//	double straightLength;
//
//	switch (_pathType) {
//	case pathTypeEnum::LSL:
//		// berechne zunächst den kürzesten LSL Anflug
//		// der erste Kreismittelpunkt kann direkt berechnet werden
//		circleCenter_t[0].x = start_t.x
//				- circleRadius * sin(to_radians(90 - startHead_t));
//		circleCenter_t[0].y = start_t.y
//				+ circleRadius * cos(to_radians(90 - startHead_t));
//		// der Mittelpunkt des zweiten Kreises kann auch direkt berechnet werden
//		// der zweite Kreis hat seinen Mittelpunkt einen Radius unterhalb und rechts des Zielpunkts
//		circleCenter_t[1].y = end_t.y + circleRadius * (-1.0);// cos(to_radians(90 - endHead_t)) == 1, da endHead_t == 270;
//		circleCenter_t[1].x = end_t.x - circleRadius * (0.0); //sin(to_radians(90 - endHead_t)) == 0, da endHead_t == 270;
//
//		// der Abstand in y-Richtung
//		deltaY = circleCenter_t[1].y - circleCenter_t[0].y;
//		deltaX = circleCenter_t[1].x - circleCenter_t[0].x;
//		// the direction between the center points of the circles
//		// fmod(..., -360) to get the left turning circle
//		headingStraight_t = fmod(to_degrees(atan2(deltaX, deltaY)), -360.0);
//		// die Rotationen in den einzelnen Kreisen
//		rotCircle[0] = fmod(headingStraight_t - startHead_t - 2 * 360, -360.0);
//		rotCircle[1] = fmod(endHead_t - headingStraight_t - 2 * 360, -360.0);
//
//		circleEntry_t[0] = start_t;
//		circleExit_t[0].x = circleCenter_t[0].x
//				+ circleRadius * cos(to_radians(-headingStraight_t));
//		circleExit_t[0].y = circleCenter_t[0].y
//				+ circleRadius * sin(to_radians(-headingStraight_t));
//		circleEntry_t[1].x = circleCenter_t[1].x
//				+ circleRadius * cos(to_radians(-headingStraight_t));
//		circleEntry_t[1].y = circleCenter_t[1].y
//				+ circleRadius * sin(to_radians(-headingStraight_t));
//		circleExit_t[1] = end_t;
//		circleEntryAngle[0] = startHeading + 90;
//		circleExitAngle[0] = startHeading + rotCircle[0] + 90;
//		circleEntryAngle[1] = endHeading - rotCircle[1] + 90;
//		circleExitAngle[1] = endHeading + 90;
//
//		//damit kann der Höhenverlust des kürzesten Weges ausgerechnet werden
//		circleLength[0] = circleRadius * to_radians(-rotCircle[0]);
//		heightLossCircle[0] = -circleLength[0] / _circleGlideRatio;
//		circleLength[1] = circleRadius * to_radians(-rotCircle[1]);
//		heightLossCircle[1] = -circleLength[1] / _circleGlideRatio;
//		inBetweenLength = sqrt(deltaX * deltaX + deltaY * deltaY);
//		heightLossStraight = -inBetweenLength / straightGlideRatio;
//
//		heightLoss = heightLossStraight + heightLossCircle[0]
//				+ heightLossCircle[1];
//
//		if (heightLoss >= requiredHeightDiff + EPS) {
//			isValidDubinsPath = false;	// we won't make it there;
//		}
//
//		// setze die Höhendaten für die einzelnen Kreispunkte
//		circleEntry_t[0].z = startCart.z;
//		circleExit_t[0].z = startCart.z - heightLossCircle[0];
//		circleEntry_t[1].z = circleExit_t[0].z - heightLossStraight;
//		circleExit_t[1].z = circleEntry_t[1].z - heightLossCircle[1];
//
////		// wie viel Höhenverlust ist verlangt
////		if(heightDiff == 0.0){	// Vergleich in dieser Form erlaubt, da nicht Ergebnis von Berechnung
////			deltaHeight = circleEntry_t[0].z - circleExit_t[1].z;
////		} else if (heightDiff < 0){
////			deltaHeight = - heightDiff;	// Angabe relativ zum Startpunkt
////			if(circleEntry_t[0].z - circleExit_t[1].z - deltaHeight < EPS){
////				// TODO Error Handling für inkonsistente Höhenangaben
////				std::cout << "ATTENTION!!! The endpoint is not reachable due to height constraints.\n";
////			}
////		} else if (heightDiff > 0){
////			deltaHeight = heightDiff;	// dieser Wert muss übereinstimmen mit start.z - end.z
////			if(startCart.z-endCart.z - deltaHeight < -EPS){
////				isValidDubinsPath = false;
////				// TODO Error Handling für inkonsistente Höhenangaben
////				std::cout << "ATTENTION!!! The endpoint is not reachable due to height constraints.\n";
////			}
////		}
////
//		//wenn das zu wenig ist, dann muss der final approach verlängert werden
//		heightLossStraight = requiredHeightDiff - heightLossCircle[0]
//				- heightLossCircle[1];	// Sollwert
//		if (heightLossStraight < -EPS) {
//			isValidDubinsPath = false;
//			// TODO Error Handling für inkonsistente Höhenangaben
//			std::cout
//					<< "ATTENTION!!! The endpoint is not reachable due to height constraints.\n";
//		}
//
//		straightLength = -(straightGlideRatio * heightLossStraight);//Sollwert
//		if (straightLength < inBetweenLength - EPS) {
//		}
//		// TODO Diese Formel ist nur gültig, wenn das Ziel auch erreichbar ist. Aufpassen wegen
//		// zwei quadratischen Lösungen
//		finalApproachLengthExt = -(deltaY * deltaY + deltaX * deltaX
//				- straightLength * straightLength)
//				/ (2 * (straightLength + deltaX));
//
//		//jetzt kann der endgültige Anflug bestimmt werden
//		circleCenterExt_t[0] = circleCenter_t[0];	// nix ändert sich
//		circleCenterExt_t[1].y = circleCenter_t[1].y;
//		circleCenterExt_t[1].x = end_t.x + finalApproachLengthExt;
//
//		deltaX = circleCenterExt_t[1].x - circleCenterExt_t[0].x;//deltaY bleibt konstant
//		// the direction between the center points of the circles
//		// fmod(..., -360) to get the left turning circle
//		headingStraight_t = fmod(to_degrees(atan2(deltaX, deltaY)), -360.0);
//		// die rotationen in den einzelnen Kreisen
//		rotCircleExt[0] = fmod(headingStraight_t - startHead_t - 2 * 360,
//				-360.0);
//		rotCircleExt[1] = fmod(endHead_t - headingStraight_t - 2 * 360, -360.0);
//		if (rotCircleExt[0] + rotCircleExt[1]
//				>= rotCircle[0] + rotCircle[1] + 350) {	//the 350 are intentional to avoid numerical problems
//				// is it is more, than it is a complete circle which is definitely >350
//				//wenn der verlängerte Anflug einen Kreis weniger enthält, dann füge den dem letzten Kreis hinzu
//			rotCircleExt[1] -= 360;
//		}
//
//		circleEntryExt_t[0] = start_t;
//		circleExitExt_t[0].x = circleCenterExt_t[0].x
//				+ circleRadius * cos(to_radians(-headingStraight_t));
//		circleExitExt_t[0].y = circleCenterExt_t[0].y
//				+ circleRadius * sin(to_radians(-headingStraight_t));
//		circleEntryExt_t[1].x = circleCenterExt_t[1].x
//				+ circleRadius * cos(to_radians(-headingStraight_t));
//		circleEntryExt_t[1].y = circleCenter_t[1].y
//				+ circleRadius * sin(to_radians(-headingStraight_t));
//		circleExitExt_t[1].x = end_t.x + finalApproachLengthExt;
//		circleExitExt_t[1].y = end_t.y;
//		circleEntryAngleExt[0] = startHeading + 90;
//		circleExitAngleExt[0] = startHeading + rotCircleExt[0] + 90;
//		circleEntryAngleExt[1] = endHeading - rotCircleExt[1] + 90;
//		circleExitAngleExt[1] = endHeading + 90;
//
//		circleLengthExt[0] = circleRadius * to_radians(-rotCircleExt[0]);
//		circleLengthExt[1] = circleRadius * to_radians(-rotCircleExt[1]);
//		inBetweenLengthExt = sqrt(deltaX * deltaX + deltaY * deltaY);
//		heightLossStraight = inBetweenLengthExt / straightGlideRatio;
//		circleEntryExt_t[0].z = startCart.z;
//		circleExitExt_t[0].z = startCart.z
//				- circleLengthExt[0] / circleGlideRatio;
//		circleEntryExt_t[1].z = circleExitExt_t[0].z - heightLossStraight;
//		circleExitExt_t[1].z = circleEntryExt_t[1].z
//				- circleLengthExt[1] / circleGlideRatio;
//		break;
//	case pathTypeEnum::RSR:
////		// berechne zunächst den kürzesten RSR Anflug
////		// der erste Kreismittelpunkt kann direkt berechnet werden
////		circleCenter_t[0].x = start_t.x
////				- circleRadius * sin(to_radians(-90 - startHead_t));
////		circleCenter_t[0].y = start_t.y
////				+ circleRadius * cos(to_radians(-90 - startHead_t));
////		// der Mittelpunkt des zweiten Kreises kann auch direkt berechnet werden
////		// der zweite Kreis hat seinen Mittelpunkt einen Radius oberhalb und rechts des Zielpunkts
////		circleCenter_t[1].y = end_t.y
////				+ circleRadius * (+1.0);// cos(to_radians(90 - endHead_t)) == 1, da endHead_t == 270;
////		circleCenter_t[1].x = end_t.x
////				- circleRadius * (0.0); //sin(to_radians(90 - endHead_t)) == 0, da endHead_t == 270;
////
////		// der Abstand in y-Richtung
////		deltaY = circleCenter_t[1].y - circleCenter_t[0].y;
////		deltaX = circleCenter_t[1].x - circleCenter_t[0].x;
////		// the direction between the center points of the circles
////		// fmod(..., +360) to get the right turning circle
////		headingStraight_t = fmod(to_degrees(atan2(deltaX, deltaY)), 360.0);
////		// die Rotationen in den einzelnen Kreisen
////		rotCircle[0] = fmod(headingStraight_t - startHead_t + 2 * 360, 360.0);
////		rotCircle[1] = fmod(endHead_t - headingStraight_t + 2 * 360, 360.0);
////
////		circleEntry_t[0] = start_t;
////		circleExit_t[0].x = circleCenter_t[0].x - circleRadius*cos(to_radians(-headingStraight_t));
////		circleExit_t[0].y = circleCenter_t[0].y - circleRadius*sin(to_radians(-headingStraight_t));
////		circleEntry_t[1].x = circleCenter_t[1].x - circleRadius*cos(to_radians(-headingStraight_t));
////		circleEntry_t[1].y = circleCenter_t[1].y - circleRadius*sin(to_radians(-headingStraight_t));
////		circleExit_t[1] = end_t;
////		circleEntryAngle[0] = startHeading - 90;
////		circleExitAngle[0] = startHeading + rotCircle[0] - 90;
////		circleEntryAngle[1] = endHeading - rotCircle[1] - 90;
////		circleExitAngle[1] = endHeading - 90;
////
////		//damit kann der Höhenverlust des kürzesten Weges ausgerechnet werden
////		circleLength[0] = circleRadius*to_radians(+rotCircle[0]);
////		heightLossCircle[0] = circleLength[0]/circleGlideRatio;
////		circleLength[1] = circleRadius*to_radians(+rotCircle[1]);
////		heightLossCircle[1] = circleLength[1]/circleGlideRatio;
////		inBetweenLength = sqrt(deltaX*deltaX + deltaY*deltaY);
////		heightLossStraight = inBetweenLength/straightGlideRatio;
////
////		// setze die Höhendaten für die einzelnen Kreispunkte
////		circleEntry_t[0].z = startCart.z;
////		circleExit_t[0].z = startCart.z - heightLossCircle[0];
////		circleEntry_t[1].z = circleExit_t[0].z - heightLossStraight;
////		circleExit_t[1].z = circleEntry_t[1].z - heightLossCircle[1];
////
////		// wie viel Höhenverlust ist verlangt
////		if(heightDiff == 0.0){	// Vergleich in dieser Form erlaubt, da nicht Ergebnis von Berechnung
////			minHeightLoss = circleEntry_t[0].z - circleExit_t[1].z;
////		} else if (heightDiff < 0){
////			minHeightLoss = - heightDiff;	// Angabe relativ zum Startpunkt
////			if(circleEntry_t[0].z - circleExit_t[1].z - minHeightLoss < EPS){
////				isValidDubinsPath = false;
////				// TODO Error Handling für inkonsistente Höhenangaben
////				std::cout << "ATTENTION!!! The endpoint is not reachable due to height constraints.\n";
////			}
////		} else if (heightDiff > 0){
////			minHeightLoss = heightDiff;	// dieser Wert muss übereinstimmen mit start.z - end.z
////			if(startCart.z-endCart.z - minHeightLoss < EPS){
////				isValidDubinsPath = false;
////				// TODO Error Handling für inkonsistente Höhenangaben
////				std::cout << "ATTENTION!!! The endpoint is not reachable due to height constraints.\n";
////			}
////		}
////
////		//wenn das zu wenig ist, dann muss der final approach verlängert werden
////		heightLossStraight = minHeightLoss - heightLossCircle[0] - heightLossCircle[1];	// Sollwert
////		if(heightLossStraight < 0.0){
////			isValidDubinsPath = false;
////			// TODO Error Handling für inkonsistente Höhenangaben
////			std::cout << "ATTENTION!!! The endpoint is not reachable due to height constraints.\n";
////		}
////		straightLength = straightGlideRatio * heightLossStraight;	//Sollwert
////		finalApproachLengthExt = -(deltaY * deltaY + deltaX * deltaX
////				- straightLength * straightLength)
////				/ (2 * (straightLength + deltaX));
////
////		//jetzt kann der endgültige Anflug bestimmt werden
////		circleCenterExt_t[0] = circleCenter_t[0];	// nix ändert sich
////		circleCenterExt_t[1].y = circleCenter_t[1].y;
////		circleCenterExt_t[1].x = end_t.x + finalApproachLengthExt;
////
////		deltaX = circleCenterExt_t[1].x - circleCenterExt_t[0].x;	//deltaY bleibt konstant
////		// the direction between the center points of the circles
////		// fmod(..., +360) to get the right turning circle
////		headingStraight_t = fmod(to_degrees(atan2(deltaX, deltaY)), 360.0);
////		// die rotationen in den einzelnen Kreisen
////		rotCircleExt[0] = fmod(headingStraight_t - startHead_t + 2 * 360, +360.0);
////		rotCircleExt[1] = fmod(endHead_t - headingStraight_t + 2 * 360, +360.0);
////		if(rotCircleExt[0]+rotCircleExt[1] <= rotCircle[0]+rotCircle[1] - 350){	//the 350 are intentional to avoid numerical problems
////			// is it is more, than it is a complete circle which is definitely >350
////			//wenn der verlängerte Anflug einen Kreis weniger enthält, dann füge den dem letzten Kreis hinzu
////			rotCircleExt[1] += 360;
////		}
////
////		circleEntryExt_t[0] = start_t;
////		circleExitExt_t[0].x = circleCenterExt_t[0].x - circleRadius*cos(to_radians(-headingStraight_t));
////		circleExitExt_t[0].y = circleCenterExt_t[0].y - circleRadius*sin(to_radians(-headingStraight_t));
////		circleEntryExt_t[1].x = circleCenterExt_t[1].x - circleRadius*cos(to_radians(-headingStraight_t));
////		circleEntryExt_t[1].y = circleCenter_t[1].y - circleRadius*sin(to_radians(-headingStraight_t));
////		circleExitExt_t[1].x = end_t.x + finalApproachLengthExt;
////		circleExitExt_t[1].y = end_t.y;
////		circleEntryAngleExt[0] = startHeading - 90;
////		circleExitAngleExt[0] = startHeading + rotCircleExt[0] - 90;
////		circleEntryAngleExt[1] = endHeading - rotCircleExt[1] - 90;
////		circleExitAngleExt[1] = endHeading - 90;
////
////		circleLengthExt[0] = circleRadius*to_radians(+rotCircleExt[0]);
////		circleLengthExt[1] = circleRadius*to_radians(+rotCircleExt[1]);
////		inBetweenLengthExt = sqrt(deltaX*deltaX + deltaY*deltaY);
////		heightLossStraight = inBetweenLengthExt/straightGlideRatio;
////		circleEntryExt_t[0].z = startCart.z;
////		circleExitExt_t[0].z = startCart.z - circleLengthExt[0]/circleGlideRatio;
////		circleEntryExt_t[1].z = circleExitExt_t[0].z - heightLossStraight;
////		circleExitExt_t[1].z = circleEntryExt_t[1].z - circleLengthExt[1]/circleGlideRatio;
////		break;
//	default:
//		break;
//	}
//
//	//now unrotate the coordinate system
//	Position_Cartesian temp;
//	rotatePoint(circleEntry_t[0].x, circleEntry_t[0].y, -rotation, temp.x,
//			temp.y);
//	circleEntry[0] = Position(end, temp);
//	rotatePoint(circleExit_t[0].x, circleExit_t[0].y, -rotation, temp.x,
//			temp.y);
//	circleExit[0] = Position(end, temp);
//	rotatePoint(circleEntry_t[1].x, circleEntry_t[1].y, -rotation, temp.x,
//			temp.y);
//	circleEntry[1] = Position(end, temp);
//	rotatePoint(circleExit_t[1].x, circleExit_t[1].y, -rotation, temp.x,
//			temp.y);
//	circleExit[1] = Position(end, temp);
//
//	rotatePoint(circleCenter_t[0].x, circleCenter_t[0].y, -rotation, temp.x,
//			temp.y);
//	circleCenter[0] = Position(end, temp);
//	rotatePoint(circleCenter_t[1].x, circleCenter_t[1].y, -rotation, temp.x,
//			temp.y);
//	circleCenter[1] = Position(end, temp);
//
//	// and now the extended path
//
//	rotatePoint(circleEntryExt_t[0].x, circleEntryExt_t[0].y, -rotation, temp.x,
//			temp.y);
//	circleEntryExt[0] = Position(end, temp);
//	rotatePoint(circleExitExt_t[0].x, circleExitExt_t[0].y, -rotation, temp.x,
//			temp.y);
//	circleExitExt[0] = Position(end, temp);
//	rotatePoint(circleEntryExt_t[1].x, circleEntryExt_t[1].y, -rotation, temp.x,
//			temp.y);
//	circleEntryExt[1] = Position(end, temp);
//	rotatePoint(circleExitExt_t[1].x, circleExitExt_t[1].y, -rotation, temp.x,
//			temp.y);
//	circleExitExt[1] = Position(end, temp);
//
//	rotatePoint(circleCenterExt_t[0].x, circleCenterExt_t[0].y, -rotation,
//			temp.x, temp.y);
//	circleCenterExt[0] = Position(end, temp);
//	rotatePoint(circleCenterExt_t[1].x, circleCenterExt_t[1].y, -rotation,
//			temp.x, temp.y);
//	circleCenterExt[1] = Position(end, temp);
//
//	//set everything else in the DubinsPath object
//	startPoint = start;
//	endPoint = end;
//
////	cachesDirty = false;
//
//	//TODO Debug ausgabe entfernen!
////	std::cout << asJson().dump(4) << std::endl;
//}
//

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

