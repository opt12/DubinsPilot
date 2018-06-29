/*
 * Utils.h
 *
 *  Created on: 20.06.2018
 *      Author: eckstein
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <math.h>       /* M_PI */


double to_degrees(const double radians);
double deg2rad(const double deg);
double to_radians(const double degrees);
double knots_to_ms(const double valKnots);

//rotates a coordinate pair (x, y) by rotDeg degrees around the origin (0,0)
// positive rotDeg rotates clockwise
void rotatePoint(const double x, const double y, const double rotDeg, double& outX, double& outY);

#endif /* UTILS_H_ */
