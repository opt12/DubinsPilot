/*
 * FlightPhase.h
 *
 *  Created on: 13.07.2018
 *      Author: eckstein
 */

#ifndef FLIGHTPHASE_H_
#define FLIGHTPHASE_H_
#include <QObject>

#include <vector>
#include "DataCenter.h"
#include "auto_pilot.h"
#include "enumDeclarations.h"
#include "Position.h"

class FlightPhase : public QObject {

	Q_OBJECT

public:
	FlightPhase(QObject* parent = 0);
	virtual~FlightPhase();

signals:
	void sigCtrlActiveStateChanged(ctrlType ctrl, bool active);
	void sigRequestedSetValueChanged(ctrlType ctrl, double setValue, bool forwardToGui, bool isLeftCircle = true);
	void sigCircleDirectionChanged(bool isLeftCircle, double radius);
	void sigDisplayFlightPhase(QString flightPhase, QString toGo);

//	void sigStartSegStats(bool isActive);



public:
	virtual void performFlight(void) = 0;
	virtual bool isFinished(void) = 0;

	void setSegmentType(segmentTypeEnum _segType){
		segType = _segType;
	}

	segmentTypeEnum getSegType() const {
		return segType;
	}

protected:
	static DataCenter *dc;
	boolean debug = false;
	Position_Cartesian initialDisplacement;

//	Position startPosition;
//	QTime startTime = QTime();
//	Position lastPosition;
//	Position_Cartesian lastWindDisplacement;
//	double cartesianPathLengthEarthFrame = 0.0; //holds the Integral of the path in the xy-plane in meters
//	double cartesianPathLengthWindFrame = 0.0; //holds the Integral of the path in the xy-plane in meters
//	double heightLoss = 0.0;	//holds the height loss of this segment in meters
//	double flightTime = 0.0;	//holds the flight time of this segment in seconds;
	segmentTypeEnum segType = segmentTypeEnum::INVALID;

	void updateSegmentData(void);
};

class CirclePhase: public FlightPhase {
public:
	CirclePhase(Position _circleCenter, double _radius,
			double _angleIn, double _angleOut, QObject* parent = 0);
	~CirclePhase();

	void performFlight(void);
	bool isFinished(void) {
		return isCircleFinished;
	}

private:
	Position circleCenter;
	double radius;
	double angleIn, angleOut;
	double angleFlown=0.0, lastAngle = 0.0;
	bool isCircleStarted = false, isCircleFinished = false;
};

class StraightPhase : public FlightPhase {
public:
	StraightPhase(Position _start, Position _end,
			double _radiusForOvershoot, Position _nextCircleCenter,
			QObject* parent = 0);
	~StraightPhase();

	void performFlight(void);
	bool isFinished(void) {
		return isStraightFinished;
	}

private:
	Position start, end;
	double tangentialHeading;
	Position nextCircleCenter;
	double radiusForOvershoot;
	bool isStraightStarted = false, isStraightFinished = false;
};

class RunOutPhase : public FlightPhase{
public:
	RunOutPhase(Position _elf, double _elfHeading, QObject* parent = 0);
	~RunOutPhase();

	void performFlight(void);
	bool isFinished(void) {
		return true;
	}

private:
	Position elf;
	double elfHeading;
};

#endif /* FLIGHTPHASE_H_ */
