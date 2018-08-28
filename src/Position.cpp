/*
 * Position.cpp
 *
 *  Created on: 26.05.2018
 *      Author: eckstein
 */

#include "Position.h"

#include <cmath>
#include <iostream>
using namespace GeographicLib;

//initialize static members
Position Position::origin = Position(0.0, 0.0);
Geocentric Position::earth = Geocentric(Constants::WGS84_a(),
		Constants::WGS84_f());
LocalCartesian Position::proj = LocalCartesian(origin.pos.lati, origin.pos.longi, origin.pos.altitude,
		earth);

double Position::getDistanceCart(const Position pointB) const {
	LocalCartesian tempProj = LocalCartesian(pos.lati, pos.longi, pos.altitude,earth);

	double x, y, z;
	tempProj.Forward(pointB.pos.lati, pointB.pos.longi, pointB.pos.altitude, x, y, z);
	return sqrt(x*x +y*y);	//altitude difference is neglected as only the straight glide path is relevant
}

Position_Cartesian Position::getCartesianDifference(const Position pointB) const {
	LocalCartesian tempProj = LocalCartesian(pos.lati, pos.longi, pos.altitude,earth);

	double x, y, z;
	tempProj.Forward(pointB.pos.lati, pointB.pos.longi, pointB.pos.altitude, x, y, z);
	return Position_Cartesian(x, y, z);
}

double Position::getHeadingCart(const Position pointB) const {
	LocalCartesian tempProj = LocalCartesian(pos.lati, pos.longi, pos.altitude,earth);

	double x, y, z;
	tempProj.Forward(pointB.pos.lati, pointB.pos.longi, pointB.pos.altitude, x, y, z);
	double headingRad = -atan2(y, x) + M_PI/2;
	return to_degrees(headingRad);
}

Position_Cartesian Position::convertToCart() {
	//reference is the origin which defines Position::proj;
	double x, y, z;
	Position::proj.Forward(pos.lati, pos.longi, pos.altitude, x, y, z);
	return Position_Cartesian(x, y, z);
}

Position::~Position() {
	// TODO Auto-generated destructor stub
}

Position_Cartesian Position::getOrigin_Cart() {
	return origin.convertToCart();	//should be always Position_Cartesian(0.0, 0.0, 0.0);
}

Position_Cartesian Position::getPosition_Cart() {
	//reference is the origin which defines Position::proj;
	return this->convertToCart();
}


