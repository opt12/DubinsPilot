/*
 * constants.h
 *
 *  Created on: 22.07.2018
 *      Author: eckstein
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_
#include "utils.h"
#include <math.h>       /* M_PI */

static const double CIRCLE_RADIUS = 450.0;
constexpr static double GLIDE_ANGLE_STRAIGTH = -5.5;
constexpr static double GLIDE_ANGLE_CIRCLE = -5.5;

static constexpr double GLIDE_RATIO_STRAIGHT = 1/tan(deg2rad(GLIDE_ANGLE_STRAIGTH));
static constexpr double GLIDE_RATIO_CIRCLE = 1/tan(deg2rad(GLIDE_ANGLE_CIRCLE));

static const double EPS = 5.0;

#endif /* CONSTANTS_H_ */
