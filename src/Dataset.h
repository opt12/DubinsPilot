/*
 * Dataset.h
 *
 *  Created on: 05.05.2018
 *      Author: eckstein
 */

#ifndef SRC_DATASET_H_
#define SRC_DATASET_H_

#include <QTime>
#include <QString>
#include "Position.h"
#include "enumDeclarations.h"
#include <vector>

//#include <iostream>

class Dataset {
public:
	friend std::ostream& operator<<(std::ostream& o, const Dataset& data) {
		return o
				<< data.timestamp.toString(Qt::SystemLocaleShortDate).toStdString()
				<< ":\tspeed: " << data.indicated_airspeed
				<< " [KIAS]\tyoke-pitch: " << data.artstab_pitch_ratio * 100
				<< "\n";
	}

	QTime timestamp = QTime::currentTime();

	void setPos_WGS84(double lati, double longi, double height) {
		pos.setPosition_WGS84(lati, longi, height);
	}

	Position_WGS84 getPosition_WGS84() {
		Position_WGS84 posi = pos.getPosition_WGS84();
		return posi;
	}

	Position_Cartesian getPosition_Cart() {
		return pos.getPosition_Cart();
	}

	static QString csvHeading();
	QString csvDataRecord();

	//statistics
	int counter=0;	//to count the received UDP datasets since last write
	//plane Data
	double indicated_airspeed = 0.0;	//kias
	double true_airspeed = 0.0; //knots
	double groundspeed = 0.0; //knots

	double vh_ind_fpm = 0.0; // fpm
	double vh_ind = 0.0; // m/sec

	double vpath = 0.0, alpha = 0.0, true_theta = 0.0; // deg
	double true_phi = 0.0, true_psi = 0.0; // deg

	double altitude_ft_pilot = 0.0; //ft
	double y_agl = 0.0; //meter

//	double latitude = 0.0, longitude = 0.0; //WGS84
//	double elevation = 0.0; // WGS84

	/*************************/

	double roll_electric_deg_pilot = 0.0;
	double heightOverGnd = 0.0;
	Position pos = Position(0.0, 0.0);

	//controller data
	bool climbControlActive = false;
	bool rollControlActive = false;
	double requestedClimbRate = 0.0;
	double requestedRoll = 0.0;
	double PSpeed = 0.0, ISpeed = 0.0, DSpeed = 0.0;
	double PRoll = 0.0, IRoll = 0.0, DRoll = 0.0;
	double artstab_pitch_ratio = 0.0;
	double artstab_roll_ratio = 0.0;

	class PidParams{
	public:
		double p=0.0, i=0.0, d=0.0;
	};

	std::vector<PidParams> pidParameters[ctrlType::_size()];

};

#endif /* SRC_DATASET_H_ */
