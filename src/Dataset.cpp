/*
 * Dataset.cpp
 *
 *  Created on: 05.05.2018
 *      Author: eckstein
 */

#include "Dataset.h"

QString Dataset::csvHeading() {
	return "timestamp;"
			"counter;"
			"indicated_airspeed;"
//			"indicated_airspeed2;"
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
//			+ QString::number(indicated_airspeed2, 'g', 6) + ";"
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

json Dataset::asJson() {
	// create an empty structure (null)
	json j;

	j["timestamp"] = QTime::currentTime().toString("hh:mm:ss.zzz").toStdString().c_str();
	j["counter"] = counter;
	j["indicated_airspeed"] = indicated_airspeed;
//	j["indicated_airspeed2"] = indicated_airspeed2;
	j["true_airspeed"] = true_airspeed;
	j["groundspeed"] = groundspeed;
	j["vh_ind_fpm"] = vh_ind_fpm;
	j["vh_ind"] = vh_ind;
	j["vpath"] = vpath;
	j["alpha"] = alpha;
	j["true_theta"] = true_theta;
	j["true_phi"] = true_phi;
	j["true_psi"] = true_psi;
	j["altitude_ft_pilot"] = altitude_ft_pilot;
	j["y_agl"] = y_agl;
	j["pos_WGS84"] = pos.getPosition_WGS84().asJson();
	j["roll_electric_deg_pilot"] = roll_electric_deg_pilot;
	j["heightOverGnd"] = heightOverGnd;
	j["requestedClimbRate"] = requestedClimbRate;
	j["requestedRoll"] = requestedRoll;
	j["artstab_pitch_ratio"] = artstab_pitch_ratio;
	j["artstab_roll_ratio"] = artstab_roll_ratio;
	j["pos_Cart"] = pos.getPosition_Cart().asJson();

	return j;
}

