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



public:
	virtual void performCommand(void) = 0;
	virtual bool isFinished(void) = 0;

	void setSegmentType(segmentTypeEnum _segType){
		segType = _segType;
	}

protected:
	static DataCenter *dc;
	Position_Cartesian initialDisplacement;
	segmentTypeEnum segType = segmentTypeEnum::INVALID;
};

class CirclePhase: public FlightPhase {
public:
	CirclePhase(Position _circleCenter, double _radius,
			double _angleIn, double _angleOut, QObject* parent = 0);
	~CirclePhase();

	void performCommand(void);
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

	void performCommand(void);
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

	void performCommand(void);
	bool isFinished(void) {
		return true;
	}

private:
	Position elf;
	double elfHeading;
};

#endif /* FLIGHTPHASE_H_ */
