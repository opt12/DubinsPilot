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
#include <iostream>
#include "utils.h"
#include "enumDeclarations.h"
#include "DubinsPath.h"

#include "json.hpp"
// for convenience
using json = nlohmann::json;

//#include <iostream>

class Dataset {
public:
	friend std::ostream& operator<<(std::ostream& o, const Dataset& data) {
		return o
				<< data.timestamp.toString(Qt::SystemLocaleShortDate).toStdString()
				<< ":\tspeed: " << data.indicated_airspeed_ms
				<< " [m/s]\tyoke-pitch: " << data.controllerOutputs[ctrlType::CLIMB_CONTROL] * 100
				<< "\n";
	}

	QTime timestamp = QTime::currentTime();

	void setPos_WGS84(double lati, double longi, double altitude) {
		pos.setPosition_WGS84(lati, longi, altitude);
	}

	void setElfPos_WGS84(double lati, double longi, double altitude) {
		elf.setPosition_WGS84(lati, longi, altitude);
	}

	void setElfPos_relative(Position _pos, double forward, double right, double height, double rotation){
		// ELf distance need to be transformed from Aircraft CoSy to Earth frame
		// The aircraft has a heading of true_psi
		// We need a rotation of the coordinates by -true_psi
		//TODO XXX
		double true_psi_rad = to_radians(true_psi);
		double xTransf = cos(-true_psi_rad)*right - sin(-true_psi_rad)*forward;
		double yTransf = sin(-true_psi_rad)*right + cos(-true_psi_rad)*forward;
		double hTransf = height;
		std::cout << "Coordinate Transform yielded with true_Psi= "<<
				to_degrees(true_psi_rad) <<std::endl;
		std::cout << "xTransf= "<< xTransf << "; yTransf= "<< yTransf << "; hTransf= "<< hTransf <<std::endl;
		elf = Position(_pos.getPosition_WGS84(), xTransf, yTransf, hTransf);
		std::cout <<"elf set to: "<< elf.getPosition_WGS84().asJson().dump(4) << std::endl;
		std::cout <<"elf set to: "<< elf.getPosition_Cart().asJson().dump(4) << std::endl;
		elfHeading = true_psi + rotation;
		isElfSet = true;
	}

	void setElfPos(Position_WGS84 elfPosition, double _elfHeading){
		elf = Position(elfPosition);
		elfHeading = _elfHeading;
		isElfSet = true;
	}

	void resetElfPos(void){
		isElfSet = false;
	}

	void setDubinsPath(Position start, Position end,
			double startHeading, double endHeading,
			double circleRadius,
			double circleGlideRatio, double straightGlideRatio,
			pathTypeEnum _pathType){
		db = DubinsPath(start, end, startHeading, endHeading, circleRadius,
				circleGlideRatio, straightGlideRatio, _pathType);
		isDubinsPathSet = db.getIsValidDubinsPath();
	}

	DubinsPath getDubinsPath(void) const{
		return db;
	}

	bool isValidDubinsPath(void) const {
		return db.getIsValidDubinsPath();
	}


	Position_WGS84 getPosition_WGS84() {
		Position_WGS84 posi = pos.getPosition_WGS84();
		return posi;
	}

	Position_Cartesian getPosition_Cart() {
		return pos.getPosition_Cart();
	}

	Position_WGS84 getElfPosition_WGS84() {
		return elf.getPosition_WGS84();
	}

	Position_Cartesian getElfPosition_Cart() {
		return elf.getPosition_Cart();
	}

	static QString csvHeading();
	QString csvDataRecord();
	json asJson();
	void resetWindDisplacement(void) {
		windDisplacement={0.0, 0.0, 0.0};
	}

	double getElfHeightDiff() const
	{
		return elfHeightDiff;
	}

	void setElfHeightDiff(double elfHeightDiff = 0.0)
	{
		this->elfHeightDiff = elfHeightDiff;
	}

	//statistics
	int counter=0;	//to count the received UDP datasets since last write
	//plane Data
	double indicated_airspeed_ms = 0.0;	//m/s
	double true_airspeed = 0.0; //m/s
	double groundspeed = 0.0; //m/s

	double vh_ind_fpm = 0.0; // fpm
	double vh_ind = 0.0; // m/sec

	double vpath = 0.0, alpha = 0.0, true_theta = 0.0; // deg
	double true_phi = 0.0, true_psi = 0.0; // deg

	double altitude_ft_pilot = 0.0; //ft
	double y_agl = 0.0; //meter

	double wind_direction_degt=0.0, wind_speed_ms=0.0;
	// the displacement due to wind is integrated with each wind_direction_degt update
	Position_Cartesian windDisplacement = {0.0, 0.0, 0.0};
//	double latitude = 0.0, longitude = 0.0; //WGS84
//	double elevation = 0.0; // WGS84

	/*************************/

	double roll_electric_deg_pilot = 0.0;
//	double heightOverGnd = 0.0;	//this value isn't in the datarefs
//	Position pos = Position(52.0, 9.0, 2000.0);
	Position pos = Position(0, 0, 2000.0);
	Position elf = Position(0.0, 0.0);
	bool isElfSet = false;
	DubinsPath db = DubinsPath();
	bool isDubinsPathSet = false;
	double elfHeading = 0.0;
	double elfHeightDiff = 0.0;

	//controller data
	bool climbControlActive = false;
	bool rollControlActive = false;
	std::vector<double> requestedTargetVals = std::vector<double>(ctrlType::_size());;
	double PSpeed = 0.0, ISpeed = 0.0, DSpeed = 0.0;
	double PRoll = 0.0, IRoll = 0.0, DRoll = 0.0;
	std::vector<double> controllerOutputs = std::vector<double>(ctrlType::_size());;
	std::vector<double> currentValue = std::vector<double>(ctrlType::_size());;

	class PidParams{
	public:
		double p=0.0, i=0.0, d=0.0;
	};

	std::vector<PidParams> pidParameters[ctrlType::_size()];

private:


};

#endif /* SRC_DATASET_H_ */
