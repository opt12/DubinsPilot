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
			"indicated_airspeed_ms;"
//			"indicated_airspeed2;"
			"true_airspeed;"
			"groundspeed;"
			"vh_ind_fpm;"
			"vh_ind;"
			"glideRatio;"
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
			"requestedClimbRate;"
			"measuredClimbRate;"
			"ElevatorOutput;"
			"requestedRoll;"
			"measuredRoll;"
			"AileronOutput;"
			"requestedHeading;"
			"measuredHeading;"
			"HeadingRollOutput;"
			"requestedRadius;"
			"measuredRadius;"
			"RadiusCorrectionOutput;"
			"x_cart;"
			"y_cart;"
			"z_cart;"
			"wind_direction_degt;"
			"wind_speed_ms;"
			"x_windDisplacement;"
			"y_windDisplacement;"
			"\n";
}

QString Dataset::csvDataRecord() {
	return QTime::currentTime().toString("hh:mm:ss.zzz") + ";"
			+ QString::number(counter, 'f', 8) + ";"
			+ QString::number(indicated_airspeed_ms, 'g', 6) + ";"
			+ QString::number(true_airspeed, 'g', 6) + ";"
			+ QString::number(groundspeed, 'g', 6) + ";"
			+ QString::number(vh_ind_fpm, 'g', 6) + ";"
			+ QString::number(vh_ind, 'g', 6) + ";"
			+ QString::number(true_airspeed/vh_ind, 'g', 6) + ";"
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
			+ QString::number(requestedTargetVals[ctrlType::CLIMB_CONTROL], 'g', 6) + ";"
			+ QString::number(currentValue[ctrlType::CLIMB_CONTROL], 'g', 6) + ";"
			+ QString::number(controllerOutputs[ctrlType::CLIMB_CONTROL], 'g', 6) + ";"
			+ QString::number(requestedTargetVals[ctrlType::ROLL_CONTROL], 'g', 6) + ";"
			+ QString::number(currentValue[ctrlType::ROLL_CONTROL], 'g', 6) + ";"
			+ QString::number(controllerOutputs[ctrlType::ROLL_CONTROL], 'g', 6) + ";"
			+ QString::number(requestedTargetVals[ctrlType::HEADING_CONTROL], 'g', 6) + ";"
			+ QString::number(currentValue[ctrlType::HEADING_CONTROL], 'g', 6) + ";"
			+ QString::number(controllerOutputs[ctrlType::HEADING_CONTROL], 'g', 6) + ";"
			+ QString::number(requestedTargetVals[ctrlType::RADIUS_CONTROL], 'g', 6) + ";"
			+ QString::number(currentValue[ctrlType::RADIUS_CONTROL], 'g', 6) + ";"
			+ QString::number(controllerOutputs[ctrlType::RADIUS_CONTROL], 'g', 6) + ";"
			+ QString::number(pos.getPosition_Cart().x, 'g', 8) + ";"
			+ QString::number(pos.getPosition_Cart().y, 'g', 8) + ";"
			+ QString::number(pos.getPosition_Cart().z, 'g', 8) + ";"
			+ QString::number(wind_direction_degt, 'g', 6) + ";"
			+ QString::number(wind_speed_ms, 'g', 6) + ";"
			+ QString::number(windDisplacement.x, 'g', 12) + ";"
			+ QString::number(windDisplacement.y, 'g', 12) + ";" + "\n";
}

json Dataset::asJson() {
	// create an empty structure (null)
	json j;

	j["timestamp"] = QTime::currentTime().toString("hh:mm:ss.zzz").toStdString().c_str();
	j["counter"] = counter;
	j["indicated_airspeed_ms"] = indicated_airspeed_ms;
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
//	j["heightOverGnd"] = heightOverGnd;
	j["requestedClimbRate"] = requestedTargetVals[ctrlType::CLIMB_CONTROL];
	j["requestedRoll"] = requestedTargetVals[ctrlType::ROLL_CONTROL];
	j["artstab_pitch_ratio"] = controllerOutputs[ctrlType::CLIMB_CONTROL];
	j["artstab_roll_ratio"] = controllerOutputs[ctrlType::ROLL_CONTROL];
	j["pos_Cart"] = pos.getPosition_Cart().asJson();
	j["wind_direction_degt"] = wind_direction_degt;
	j["wind_speed_kt"] = wind_speed_ms;
	j["windDisplacement"] = windDisplacement.asJson();




	return j;
}

