/*
 * Utils.cpp
 *
 *  Created on: 20.06.2018
 *      Author: eckstein
 */

#include <utils.h>


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

double ms_to_knots(const double valKnots){
	return valKnots*1.9438444924574;
}

double ft_to_m(const double valFt){
	return valFt*0.304803706;
}


//taken from https://rosettacode.org/wiki/Angle_difference_between_two_bearings#C.2B.2B
double getAngularDifference(double b1, double b2) {
	double r = fmod(b2 - b1, 360.0);
	if (r < -180.0)
		r += 360.0;
	if (r >= 180.0)
		r -= 360.0;
	return r;
}

double getRollAngle(double tas, double radius){
	//see Clancy, L.J, Equation 14.9; https://en.wikipedia.org/wiki/Banked_turn
	// r = \frac{v²}{g \tan \theta}
	// \tan \theta = \frac{v²}{gr}
	const double g = 9.81;
	return to_degrees(atan(tas*tas/(g*radius)));
}


