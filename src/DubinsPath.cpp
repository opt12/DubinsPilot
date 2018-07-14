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
			<< to_radians(circleExitAngle[0]) << ", " << tmp1.x << ", " << tmp1.y << ", "
			<< circleRadius << ")"<<std::endl;
	Position_Cartesian tmp2 = circleCenter[1].getPosition_Cart();
	matlabCommand << "plot_arc_cw(" << to_radians(circleEntryAngle[1]) << ", "
			<< to_radians(circleExitAngle[1]) << ", " << tmp2.x << ", " << tmp2.y << ", "
			<< circleRadius << ")" << std::endl;
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

std::string DubinsPath::getExtendedPathAsMatlabCommand(void) {
	std::stringstream matlabCommand;
	Position_Cartesian tmp1 = circleCenterExt[0].getPosition_Cart();
	matlabCommand << "plot_arc_cw(" << to_radians(circleEntryAngleExt[0]) << ", "
			<< to_radians(circleExitAngleExt[0]) << ", " << tmp1.x << ", " << tmp1.y << ", "
			<< circleRadius << ")"<<std::endl;
	Position_Cartesian tmp2 = circleCenterExt[1].getPosition_Cart();
	matlabCommand << "plot_arc_cw(" << to_radians(circleEntryAngleExt[1]) << ", "
			<< to_radians(circleExitAngleExt[1]) << ", " << tmp2.x << ", " << tmp2.y << ", "
			<< circleRadius << ")" << std::endl;
	tmp1 = circleExitExt[0].getPosition_Cart();
	tmp2 = circleEntryExt[1].getPosition_Cart();
	matlabCommand << "line([" << tmp1.x << " " << tmp2.x << "], [" << tmp1.y
			<< " " << tmp2.y << "])" << std::endl;
	tmp1 = circleExitExt[1].getPosition_Cart();
	tmp2 = endPoint.getPosition_Cart();
	matlabCommand << "line([" << tmp1.x << " " << tmp2.x << "], [" << tmp1.y
			<< " " << tmp2.y << "])" << std::endl;
	tmp1 = circleEntryExt[0].getPosition_Cart();
//	tmp2 = endPoint.getPosition_Cart();
	matlabCommand << "line([" << tmp1.x << " " << tmp2.x << "], [" << tmp1.y
			<< " " << tmp2.y << "])" << std::endl;

	return matlabCommand.str();
}

void DubinsPath::updateCaches(void) {
	//updates all the cached values whenever a value of circle one is changed
	// coordinates around a center with given angles is given as
	// x = -r*sin(-theta) + xc;
	// y = r*cos(-theta) + yc;
	Position_Cartesian tmp1 = circleCenter[0].getPosition_Cart();
	circleEntry[0] = Position(Position::getOrigin_WGS84(),
			-circleRadius * sin(-circleEntryAngle[0]) + tmp1.x,
			-circleRadius * cos(-circleEntryAngle[0]) + tmp1.y, tmp1.z);//TODO this is not really correct
	circleExit[0] = Position(Position::getOrigin_WGS84(),
			-circleRadius * sin(-circleExitAngle[0]) + tmp1.x,
			-circleRadius * cos(-circleExitAngle[0]) + tmp1.y, tmp1.z);	//TODO this is not really correct

	Position_Cartesian tmp2 = circleCenter[1].getPosition_Cart();
	circleEntry[1] = Position(Position::getOrigin_WGS84(),
			-circleRadius * sin(-circleEntryAngle[1]) + tmp2.x,
			-circleRadius * cos(-circleEntryAngle[1]) + tmp2.y, tmp2.z);//TODO this is not really correct

	circleExit[1] = Position(Position::getOrigin_WGS84(),
			-circleRadius * sin(-circleExitAngle[1]) + tmp2.x,
			-circleRadius * cos(-circleExitAngle[1]) + tmp2.y, tmp2.z);	//TODO this is not really correct

	circleLength[0] = circleRadius * (circleExitAngle[0] - circleEntryAngle[0])
			* 2 * M_PI;
	circleLength[1] = circleRadius * (circleExitAngle[1] - circleEntryAngle[1])
			* 2 * M_PI;
	inBetweenLength = circleExit[0].getDistanceCart(circleEntry[1]);
	finalApproachLengthExt = circleExit[1].getDistanceCart(endPoint);
	cachesDirty = false;
}

void DubinsPath::calculateDubinsPath(const Position start, const Position end,
		const double startHeading, const double endHeading, const double _circleRadius,
		const double _circleGlideRatio, const double _straightGlideRatio,
		const pathTypeEnum _pathType, const double windVelocity, const double windHeading) {

	pathType = _pathType;
	// get the cartesian coordinates of the start, and endpoints
//	Position_Cartesian startCart = start.getPosition_Cart(),
//			endCart = end.getPosition_Cart();
	// get the cartesian coordinates of the start, and endpoints
	// firstly, the endpoint is used as the origin
	Position_Cartesian startCart = end.getCartesianDifference(start),
			endCart = Position_Cartesian(0, 0, 0);

	// rotate the system to have the endHeading in 270° direction (-X-axis)
	double rotation = 270.0 - endHeading;
	double startHead_t = fmod(startHeading + rotation, 360),
			endHead_t = fmod(endHeading + rotation, 360);
	Position_Cartesian start_t, end_t;
	rotatePoint(startCart.x, startCart.y, rotation, start_t.x, start_t.y);
	start_t.z = startCart.z;
	rotatePoint(endCart.x, endCart.y, rotation, end_t.x, end_t.y);
	end_t.z = endCart.z;

	double deltaHeight = startCart.z - endCart.z;

	//start with the calculation of the shortest dubins path:
	Position_Cartesian circleCenter_t[2], circleEntry_t[2], circleExit_t[2];
	Position_Cartesian circleCenterExt_t[2], circleEntryExt_t[2], circleExitExt_t[2];
	double deltaX, deltaY;
	double headingStraight_t;
	double rotCircle[2];
	double rotCircleExt[2];
	double heightLossCircle, heightLossStraight;
	double straightLength;

	switch (_pathType) {
	case pathTypeEnum::LSL:
		// berechne zunächst den kürzesten LSL Anflug
		// der erste Kreismittelpunkt kann direkt berechnet werden
		circleCenter_t[0].x = start_t.x
				- _circleRadius * sin(to_radians(90 - startHead_t));
		circleCenter_t[0].y = start_t.y
				+ _circleRadius * cos(to_radians(90 - startHead_t));
		// der Mittelpunkt des zweiten Kreises kann auch direkt berechnet werden
		// der zweite Kreis hat seinen Mittelpunkt einen Radius unterhalb und rechts des Zielpunkts
		circleCenter_t[1].y = end_t.y
				+ _circleRadius * (-1.0);// cos(to_radians(90 - endHead_t)) == 1, da endHead_t == 270;
		circleCenter_t[1].x = end_t.x
				- _circleRadius * (0.0); //sin(to_radians(90 - endHead_t)) == 0, da endHead_t == 270;

		// der Abstand in y-Richtung
		deltaY = circleCenter_t[1].y - circleCenter_t[0].y;
		deltaX = circleCenter_t[1].x - circleCenter_t[0].x;
		// the direction between the center points of the circles
		// fmod(..., -360) to get the left turning circle
		headingStraight_t = fmod(to_degrees(atan2(deltaX, deltaY)), -360.0);
		// die rotationen in den einzelnen Kreisen
		rotCircle[0] = fmod(headingStraight_t - startHead_t - 2 * 360, -360.0);
		rotCircle[1] = fmod(endHead_t - headingStraight_t - 2 * 360, -360.0);

		circleEntry_t[0] = start_t;
		circleExit_t[0].x = circleCenter_t[0].x + _circleRadius*cos(to_radians(-headingStraight_t));
		circleExit_t[0].y = circleCenter_t[0].y + _circleRadius*sin(to_radians(-headingStraight_t));
		circleEntry_t[1].x = circleCenter_t[1].x + _circleRadius*cos(to_radians(-headingStraight_t));
		circleEntry_t[1].y = circleCenter_t[1].y + _circleRadius*sin(to_radians(-headingStraight_t));
		circleExit_t[1] = end_t;
		circleEntryAngle[0] = startHeading + 90;
		circleExitAngle[0] = startHeading + rotCircle[0] + 90;
		circleEntryAngle[1] = endHeading - rotCircle[1] + 90;
		circleExitAngle[1] = endHeading + 90;

		//damit kann der Höhenverlust des kürzesten Weges ausgerechnet werden
		circleLength[0] = circleRadius*to_radians(-rotCircle[0]);
		circleLength[1] = circleRadius*to_radians(-rotCircle[1]);
		heightLossCircle = (circleLength[0]+ circleLength[1])/_circleGlideRatio;
		inBetweenLength = sqrt(deltaX*deltaX + deltaY*deltaY);
		heightLossStraight = inBetweenLength/_straightGlideRatio;

		circleEntry_t[0].z = startCart.z;
		circleExit_t[0].z = startCart.z - circleLength[0]/_circleGlideRatio;
		circleEntry_t[1].z = circleExit_t[0].z - heightLossStraight;
		circleExit_t[1].z = circleEntry_t[1].z - circleLength[1]/_circleGlideRatio;

		//wenn das zu wenig ist, dann muss der final approach verlängert werden
		heightLossStraight = deltaHeight - heightLossCircle;	// Sollwert
		straightLength = _straightGlideRatio * heightLossStraight;	//sollwert
		finalApproachLengthExt = -(deltaY * deltaY + deltaX * deltaX
				- straightLength * straightLength)
				/ (2 * (straightLength + deltaX));

		//jetzt kann der endgültige Anflug bestimmt werden
		circleCenterExt_t[0] = circleCenter_t[0];	// nix ändert sich
		circleCenterExt_t[1].y = circleCenter_t[1].y;
		circleCenterExt_t[1].x = end_t.x + finalApproachLengthExt;

		deltaX = circleCenterExt_t[1].x - circleCenterExt_t[0].x;	//deltaY bleibt konstant
		// the direction between the center points of the circles
		// fmod(..., -360) to get the left turning circle
		headingStraight_t = fmod(to_degrees(atan2(deltaX, deltaY)), -360.0);
		// die rotationen in den einzelnen Kreisen
		rotCircleExt[0] = fmod(headingStraight_t - startHead_t - 2 * 360, -360.0);
		rotCircleExt[1] = fmod(endHead_t - headingStraight_t - 2 * 360, -360.0);
		if(rotCircleExt[0]+rotCircleExt[1] >= rotCircle[0]+rotCircle[1] + 350){
			//wenn der verlängerte Anflug einen Kreis weniger enthält, dann füge den dem letzten Kreis hinzu
			rotCircleExt[1] -= 360;
		}

		circleEntryExt_t[0] = start_t;
		circleExitExt_t[0].x = circleCenterExt_t[0].x + _circleRadius*cos(to_radians(-headingStraight_t));
		circleExitExt_t[0].y = circleCenterExt_t[0].y + _circleRadius*sin(to_radians(-headingStraight_t));
		circleEntryExt_t[1].x = circleCenterExt_t[1].x + _circleRadius*cos(to_radians(-headingStraight_t));
		circleEntryExt_t[1].y = circleCenter_t[1].y + _circleRadius*sin(to_radians(-headingStraight_t));
		circleExitExt_t[1].x = end_t.x + finalApproachLengthExt;
		circleExitExt_t[1].y = end_t.y;
		circleEntryAngleExt[0] = startHeading + 90;
		circleExitAngleExt[0] = startHeading + rotCircleExt[0] + 90;
		circleEntryAngleExt[1] = endHeading - rotCircleExt[1] + 90;
		circleExitAngleExt[1] = endHeading + 90;

		circleLengthExt[0] = circleRadius*to_radians(-rotCircleExt[0]);
		circleLengthExt[1] = circleRadius*to_radians(-rotCircleExt[1]);
		inBetweenLengthExt = sqrt(deltaX*deltaX + deltaY*deltaY);
		heightLossStraight = inBetweenLengthExt/_straightGlideRatio;
		circleEntryExt_t[0].z = startCart.z;
		circleExitExt_t[0].z = startCart.z - circleLengthExt[0]/_circleGlideRatio;
		circleEntryExt_t[1].z = circleExitExt_t[0].z - heightLossStraight;
		circleExitExt_t[1].z = circleEntryExt_t[1].z - circleLengthExt[1]/_circleGlideRatio;
		break;
	case pathTypeEnum::RSR:
		circleCenter_t[0].x = start_t.x
				- _circleRadius * sin(to_radians(-90 - startHead_t));
		circleCenter_t[0].y = start_t.y
				+ _circleRadius * cos(to_radians(-90 - startHead_t));
		circleCenter_t[1].x = end_t.x
				- _circleRadius * sin(to_radians(-90 - endHead_t));
		circleCenter_t[1].y = end_t.y
				+ _circleRadius * cos(to_radians(-90 - endHead_t));
		deltaX = circleCenter_t[1].x - circleCenter_t[0].x;
		deltaY = circleCenter_t[1].y - circleCenter_t[0].y;
		// the direction between the center points of the circles
		// fmod(..., -360) to get the left turning circle
		headingStraight_t = fmod(to_degrees(atan2(deltaX, deltaY)), 360.0);
		rotCircle[0] = fmod(headingStraight_t - startHead_t + 2 * 360, 360.0);
		rotCircle[1] = fmod(endHead_t - headingStraight_t + 2 * 360, 360.0);
		circleEntry_t[0] = start_t;
		circleExit_t[0].x = circleCenter_t[0].x - _circleRadius*cos(to_radians(-headingStraight_t));
		circleExit_t[0].y = circleCenter_t[0].y - _circleRadius*sin(to_radians(-headingStraight_t));
		circleEntry_t[1].x = circleCenter_t[1].x - _circleRadius*cos(to_radians(-headingStraight_t));
		circleEntry_t[1].y = circleCenter_t[1].y - _circleRadius*sin(to_radians(-headingStraight_t));
		circleExit_t[1] = end_t;
		heightLossCircle = to_radians(rotCircle[0]+rotCircle[1])*_circleRadius/_circleGlideRatio;
		heightLossStraight = sqrt(deltaX*deltaX + deltaY*deltaY)/_straightGlideRatio;
		circleEntryAngle[0] = startHeading - 90;
		circleExitAngle[0] = startHeading + rotCircle[0] - 90;
		circleEntryAngle[1] = endHeading - rotCircle[1] - 90;
		circleExitAngle[1] = endHeading - 90;
		break;
	default:
		break;
	}

	//now unrotate the coordinate system
	Position_Cartesian temp;
	rotatePoint(circleEntry_t[0].x, circleEntry_t[0].y, -rotation, temp.x, temp.y);
	circleEntry[0] = Position(end, temp);
	rotatePoint(circleExit_t[0].x, circleExit_t[0].y, -rotation, temp.x, temp.y);
	circleExit[0] = Position(end, temp);
	rotatePoint(circleEntry_t[1].x, circleEntry_t[1].y, -rotation, temp.x, temp.y);
	circleEntry[1] = Position(end, temp);
	rotatePoint(circleExit_t[1].x, circleExit_t[1].y, -rotation, temp.x, temp.y);
	circleExit[1] = Position(end, temp);

	rotatePoint(circleCenter_t[0].x, circleCenter_t[0].y, -rotation, temp.x, temp.y);
	circleCenter[0] = Position(end, temp);
	rotatePoint(circleCenter_t[1].x, circleCenter_t[1].y, -rotation, temp.x, temp.y);
	circleCenter[1] = Position(end, temp);

	// and now the extended path

	rotatePoint(circleEntryExt_t[0].x, circleEntryExt_t[0].y, -rotation, temp.x, temp.y);
	circleEntryExt[0] = Position(end, temp);
	rotatePoint(circleExitExt_t[0].x, circleExitExt_t[0].y, -rotation, temp.x, temp.y);
	circleExitExt[0] = Position(end, temp);
	rotatePoint(circleEntryExt_t[1].x, circleEntryExt_t[1].y, -rotation, temp.x, temp.y);
	circleEntryExt[1] = Position(end, temp);
	rotatePoint(circleExitExt_t[1].x, circleExitExt_t[1].y, -rotation, temp.x, temp.y);
	circleExitExt[1] = Position(end, temp);

	rotatePoint(circleCenterExt_t[0].x, circleCenterExt_t[0].y, -rotation, temp.x, temp.y);
	circleCenterExt[0] = Position(end, temp);
	rotatePoint(circleCenterExt_t[1].x, circleCenterExt_t[1].y, -rotation, temp.x, temp.y);
	circleCenterExt[1] = Position(end, temp);


	//set everything else in the DubinsPath object
	startPoint = start;
	endPoint = end;

	circleRadius = _circleRadius;
	circleGlideRatio = _circleGlideRatio;
	straightGlideRatio = _straightGlideRatio;
	cachesDirty = false;

	//TODO Debug ausgabe entfernen!
//	std::cout << asJson().dump(4) << std::endl;
}

json DubinsPath::asJson(){
	json j;
	j["pathType"]=pathType._to_string();
	j["startingPoint"]= {startPoint.getPosition_WGS84().lati, startPoint.getPosition_WGS84().longi};
	j["endPoint"]= {endPoint.getPosition_WGS84().lati, endPoint.getPosition_WGS84().longi};
	j["trochoidOne"] = trochoidAsJson(circleCenter[0], circleRadius,
			circleEntryAngle[0], circleExitAngle[0]);
	j["trochoidTwo"] = trochoidAsJson(circleCenter[1], circleRadius,
			circleEntryAngle[1], circleExitAngle[1]);
	j["startHeading"] =  circleEntryAngle[0] + ((pathType == +pathTypeEnum::LSL) ? -90 : +90);
	j["endHeading"] =  circleExitAngle[1] + ((pathType == +pathTypeEnum::LSL) ? -90 : +90);
	return j;
}

json DubinsPath::trochoidAsJson(Position circleCenter, double circleRadius,
		double entryAngleDeg, double exitAngleDeg,
		Position_Cartesian displacement){
	const int DEGREE_SCALE = 10;
	json j;
	j["trochoRadius"] = circleRadius;
	j["trochoCenterStart"] = {circleCenter.getPosition_WGS84().lati, circleCenter.getPosition_WGS84().longi};
	j["trochoCenterEnd"] = {
			(circleCenter+displacement).getPosition_WGS84().lati,
			(circleCenter+displacement).getPosition_WGS84().longi
	};

	int steps = abs((exitAngleDeg-entryAngleDeg)/DEGREE_SCALE)+1;	//one point each ~5°
	double stepSize = (exitAngleDeg-entryAngleDeg)/steps;	// hw many degrees exactly, including sign
	std::vector<Position> points = std::vector<Position>(steps+1);
	for(int i=0; i<= steps; i++){
		Position_Cartesian circleRay = Position_Cartesian(0.0, circleRadius, 0.0).rotate(entryAngleDeg+i*stepSize);
		points[i] = Position(circleCenter+(i*displacement/steps), circleRay);	//TODO muss hier durch (steps+1) geteilt werden?
	}
	std::vector<std::tuple<double, double>> latlongArray;
	std::transform(points.begin(), points.end(),
			std::back_inserter(latlongArray), [](Position pos) {
				std::tuple<double, double> tup   {
					pos.getPosition_WGS84().lati,
					pos.getPosition_WGS84().longi
				};
				return tup;
			});
	j["trochoBorder"] = latlongArray;
	return j;
}


