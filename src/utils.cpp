/*
 * Utils.cpp
 *
 *  Created on: 20.06.2018
 *      Author: eckstein
 */

#include <utils.h>

double deg2rad(const double deg){
	return deg/180*(M_PI);
}

double to_degrees(const double radians) {
    return radians * (180.0 / M_PI);
}

double to_radians(const double degrees) {
    return degrees / 180.0 * M_PI;
}

void rotatePoint(const double x, const double y, const double rotDeg, double& outX, double& outY){
	outX = x* cos(to_radians(-rotDeg)) - y*sin(to_radians(-rotDeg));
	outY = x* sin(to_radians(-rotDeg)) + y*cos(to_radians(-rotDeg));
}

double knots_to_ms(const double valKnots){
	return valKnots*0.51444444444;
}

