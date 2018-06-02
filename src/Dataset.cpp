/*
 * Dataset.cpp
 *
 *  Created on: 05.05.2018
 *      Author: eckstein
 */

#include "Dataset.h"


QString  Dataset::csvHeading() {
		return "timestamp;"
				"counter;"
				"indicated_airspeed;"
				"true_airspeed;"
				"groundspeed;"
				"vh_ind_fpm;"
				"vh_ind;"
				"vpath;"
				"alpha;"
				"true_theta;"
				"true_phi;"
				"true_psi;"
				"altitude_ft_pilot;"
				"y_agl;"
				"latitude;"
				"longitude;"
				"elevation;"
				"roll_electric_deg_pilot;"
				"heightOverGnd;"
				"requestedClimbRate;"
				"requestedRoll;"
				"artstab_pitch_ratio;"
				"artstab_roll_ratio;"
				"x_cart;"
				"y_cart;"
				"z_cart;"
				"\n";
	}

QString Dataset::csvDataRecord() {
	return QTime::currentTime().toString("hh:mm:ss.zzz") + ";"
			+ QString::number(counter, 'f', 8) + ";"
			+ QString::number(indicated_airspeed, 'g', 6) + ";"
			+ QString::number(true_airspeed, 'g', 6) + ";"
			+ QString::number(groundspeed, 'g', 6) + ";"
			+ QString::number(vh_ind_fpm, 'g', 6) + ";"
			+ QString::number(vh_ind, 'g', 6) + ";"
			+ QString::number(vpath, 'g', 6) + ";"
			+ QString::number(alpha, 'g', 6) + ";"
			+ QString::number(true_theta, 'g', 6) + ";"
			+ QString::number(true_phi, 'g', 6) + ";"
			+ QString::number(true_psi, 'g', 6) + ";"
			+ QString::number(altitude_ft_pilot, 'g', 6) + ";"
			+ QString::number(y_agl, 'g', 6) + ";"
			+ QString::number(pos.getPosition_WGS84().lati, 'g', 12) + ";"
			+ QString::number(pos.getPosition_WGS84().longi, 'g', 12) + ";"
			+ QString::number(pos.getPosition_WGS84().height, 'g', 6) + ";"
			+ QString::number(roll_electric_deg_pilot, 'g', 6) + ";"
			+ QString::number(heightOverGnd, 'g', 6) + ";"
			+ QString::number(requestedClimbRate, 'g', 6) + ";"
			+ QString::number(requestedRoll, 'g', 6) + ";"
			+ QString::number(artstab_pitch_ratio, 'g', 6) + ";"
			+ QString::number(artstab_roll_ratio, 'g', 6) + ";"
			+ QString::number(pos.getPosition_Cart().x, 'g', 8) + ";"
			+ QString::number(pos.getPosition_Cart().y, 'g', 8) + ";"
			+ QString::number(pos.getPosition_Cart().z, 'g', 8) + ";" + "\n";
}

