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

#include <iostream>
#include "utils.h"

//TODO ist das der richtige Ort für die "fetten" HPP files? Andererseits kann ich sonst kein
//Geocentric erzeugen;

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

class Position_WGS84 {
public:
	Position_WGS84() {
		lati = 0.0;
		longi = 0.0, height = 0.0;
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

	json asJson() {
		json j;
		j["latitude"] = lati;
		j["longitude"] = longi;
		j["height"] = height;
		return j;
	}

	double lati, longi;
	double height;
};

class Position_Cartesian {
public:
	Position_Cartesian() {
		x = y = z = 0.0;
	}
	Position_Cartesian(double x, double y, double z = 0.0) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	json asJson() {
		json j;
		j["x"] = x;
		j["y"] = y;
		j["z"] = z;
		return j;
	}

	friend Position_Cartesian operator+(const Position_Cartesian &p1, const Position_Cartesian &p2){
		return Position_Cartesian(p1.x+p2.x, p1.y+p2.y, p1.z+p2.z);
	}

	friend Position_Cartesian operator-(const Position_Cartesian &p1, const Position_Cartesian &p2){
		return Position_Cartesian(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z);
	}

	double x = 0.0, y = 0.0, z = 0.0;
};

class Position {
public:
	Position() {
		pos.set(0.0, 0.0, 0.0);
	}

	Position(double lati, double longi, double height = 0.0) {
		pos.set(lati, longi, height);
	}

	Position(Position_WGS84 pos_WGS84) {
		Position(pos_WGS84.lati, pos_WGS84.longi, pos_WGS84.height);
	}

	Position(Position_Cartesian pos_Cart) : Position(Position::origin.pos, pos_Cart.x, pos_Cart.y, pos_Cart.z) {
		// delegating constructor
		//calls the "cartesian" constructor with with reference to the origin
		;
	}

	/** calculates a position relative to the given reference in WGS84
	 * distances are given in cartesian coordinates, height pointing upwards
	 *
	 */
	Position(const Position_WGS84 &reference, const double x, const double y, const double h) {
		GeographicLib::LocalCartesian tempProj = GeographicLib::LocalCartesian(
				reference.lati, reference.longi, reference.height, earth);

		tempProj.Reverse(x, y, h, this->pos.lati, this->pos.longi,
				this->pos.height);
//		std::cout << "Position Constructor: yielded   height: "
//				<< this->pos.height << std::endl;
	}

	friend Position operator+(const Position &pos, const Position_Cartesian &translation){
		return Position(pos.pos, translation.x, translation.y, translation.z);
	}

	friend Position operator-(const Position &pos, const Position_Cartesian &translation){
		return Position(pos.pos, -translation.x, -translation.y, -translation.z);
	}

	virtual ~Position();

	static void setOrigin(Position_WGS84 originPos) {
		origin.pos.lati = originPos.lati;
		origin.pos.longi = originPos.longi;
		origin.pos.height = originPos.height;
		proj = GeographicLib::LocalCartesian(origin.pos.lati, origin.pos.longi,
				origin.pos.height, earth);
	}

	static Position_WGS84 getOrigin_WGS84() {
		return origin.getPosition_WGS84();
	}

	Position_Cartesian getOrigin_Cart();

	Position_WGS84 getPosition_WGS84() {
		return pos;
	}

	void setPosition_WGS84(double lati, double longi, double height = 0.0) {
		pos.set(lati, longi, height);
	}

	Position_Cartesian getPosition_Cart();
	Position_Cartesian convertToCart();

	double getDistanceCart(Position pointB);
	double getHeadingCart(Position pointB);

private:
	static Position origin;
	Position_WGS84 pos;
	static GeographicLib::Geocentric earth;
	static GeographicLib::LocalCartesian proj;

};

#endif /* POSITION_H_ */
