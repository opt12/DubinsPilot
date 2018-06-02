/*
 * Position.h
 *
 *  Created on: 26.05.2018
 *      Author: eckstein
 */

#ifndef POSITION_H_
#define POSITION_H_

#include "json.hpp"
// for convenience
using json = nlohmann::json;


//TODO ist das der richtige Ort für die "fetten" HPP files? Andererseits kann ich sonst kein
//Geocentric erzeugen;

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

class Position_WGS84 {
public:
	Position_WGS84(){
		lati = 0.0; longi = 0.0, height = 0.0;
	}
	Position_WGS84(double lati, double longi, double height = 0.0) {
		this->lati = lati;
		this->longi = longi;
		this->height = height;
	}

	void set(double lati, double longi, double height = 0.0) {
		this->lati = lati;
		this->longi = longi;
		this->height = height;
	}

	json asJson(){
		json j;
		j["lati"]=lati;
		j["longi"]=longi;
		j["height"]=height;
		return j;
	}

	double lati, longi;
	double height;
};

class Position_Cartesian {
public:
	Position_Cartesian(double x, double y, double z = 0.0) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	json asJson(){
		json j;
		j["x"]=x;
		j["y"]=y;
		j["z"]=z;
		return j;
	}

	double x= 0.0, y=0.0, z=0.0;
};


class Position {
public:
	Position() {
		pos.set(0.0, 0.0, 0.0);
	}

	Position(double lati, double longi, double height = 0.0) {
		pos.set(lati, longi, height);
	}

	Position(Position_WGS84 pos_WGS84){
		Position(pos_WGS84.lati, pos_WGS84.longi, pos_WGS84.height);
	}

	Position(Position_Cartesian pos_Cart) {
		Position(convertToWGS84(pos_Cart));
	}

	virtual ~Position();

	static void setOrigin(Position_WGS84 originPos){
		origin = originPos;
		proj = GeographicLib::LocalCartesian(origin.lati, origin.longi, 0, earth);
	}

	Position_WGS84 getOrigin_WGS84(){
		return origin;
	}

	Position_Cartesian getOrigin_Cart();

	Position_WGS84 getPosition_WGS84(){
		return pos;
	}

	void setPosition_WGS84(double lati, double longi, double height=0.0){
		pos.set(lati, longi, height);
	}

	Position_Cartesian getPosition_Cart();

	double getDistanceCart(Position pointA, Position pointB);
	double getHeadingCart(Position pointA, Position pointB);


private:
	Position_WGS84 pos;
	static Position_WGS84 origin;
	static GeographicLib::Geocentric earth;
	static GeographicLib::LocalCartesian proj;

	Position_Cartesian convertToCart(Position_WGS84 pos);
	Position_WGS84 convertToWGS84(Position_Cartesian pos);

	inline double to_degrees(double radians) {
	    return radians * (180.0 / M_PI);
	}


};

#endif /* POSITION_H_ */
