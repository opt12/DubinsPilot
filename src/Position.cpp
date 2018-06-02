/*
 * Position.cpp
 *
 *  Created on: 26.05.2018
 *      Author: eckstein
 */

#include "Position.h"

#include <cmath>
using namespace GeographicLib;

//initialize static members
Position_WGS84 Position::origin = Position_WGS84(0.0, 0.0);
Geocentric Position::earth = Geocentric(Constants::WGS84_a(),
		Constants::WGS84_f());
LocalCartesian Position::proj = LocalCartesian(origin.lati, origin.longi, 0,
		earth);

Position::~Position() {
	// TODO Auto-generated destructor stub
}

Position_Cartesian Position::getOrigin_Cart() {
	return convertToCart(this->origin);	//should be always Position_Cartesian(0.0, 0.0, 0.0);
}

Position_Cartesian Position::getPosition_Cart() {
	return convertToCart(this->pos);
}

double Position::getDistanceCart(Position pointA, Position pointB) {
	Position_Cartesian pointACart = convertToCart(pointA.pos), pointBCart =
			convertToCart(pointB.pos);
	return sqrt(
			pow(pointACart.x - pointBCart.x, 2)
					+ pow(pointACart.y - pointBCart.y, 2));
}

double Position::getHeadingCart(Position pointA, Position pointB) {
	Position_Cartesian pointACart = convertToCart(pointA.pos), pointBCart =
			convertToCart(pointB.pos);
	double deltaX = pointBCart.x - pointACart.x, deltaY = pointBCart.y
			- pointACart.y;
	double headingRad = -atan2(deltaY, deltaX) + M_PI;
	return to_degrees(headingRad);
}

Position_Cartesian Position::convertToCart(Position_WGS84 pos) {
	double x, y, z;
	proj.Forward(pos.lati, pos.longi, pos.height, x, y, z);
	return Position_Cartesian(x, y, z);
}

Position_WGS84 Position::convertToWGS84(Position_Cartesian pos) {
	double lat, lon, h;
	proj.Reverse(pos.x, pos.y, pos.z, lat, lon, h);
	return Position_WGS84(lat, lon, h);
}

