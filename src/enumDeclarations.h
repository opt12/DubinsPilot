/*
 * enums.h
 *
 *  Created on: 03.06.2018
 *      Author: eckstein
 */

#ifndef ENUMDECLARATIONS_H_
#define ENUMDECLARATIONS_H_

#include"enum.h"

BETTER_ENUM(ctrlType, int, HEADING_CONTROL, RADIUS_CONTROL, ROLL_CONTROL, CLIMB_CONTROL)
BETTER_ENUM(pathTypeEnum, int, LSL, RSR, LSR, RSL)
/*the types RLR and LRL are omitted as we cannot extend their lengths to our needs;*/

#endif /* ENUMDECLARATIONS_H_ */
