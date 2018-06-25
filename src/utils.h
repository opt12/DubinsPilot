/*
 * Utils.h
 *
 *  Created on: 20.06.2018
 *      Author: eckstein
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <math.h>       /* M_PI */


double to_degrees(double radians);
double deg2rad(double deg);
double to_radians(double degrees);

//rotates a coordinate pair (x, y) by rotDeg degrees around the origin (0,0)
// positive rotDeg rotates clockwise
void rotatePoint(double x, double y, double rotDeg, double& outX, double& outY);

#endif /* UTILS_H_ */
