/*
 * DubinsScheduler.h
 *
 *  Created on: 13.07.2018
 *      Author: eckstein
 */

#ifndef DUBINSSCHEDULER_H_
#define DUBINSSCHEDULER_H_

#include <QObject>
#include <QTimer>

#include <vector>

#include <iostream>
#include "json.hpp"
// for convenience
using json = nlohmann::json;

#include "enumDeclarations.h"
#include "FlightPhase.h"
#include "DubinsPath.h"


class DubinsScheduler : public QObject {

	Q_OBJECT

public:
	DubinsScheduler(QObject* parent = 0);
	virtual ~DubinsScheduler();

public slots:
	void takeMeDown(bool active);

private slots:
	void flyThePath(void);
	void initialize(const DubinsPath* _db);

signals:
	void sigCtrlActiveStateChanged(ctrlType ctrl, bool active);
	void sigRequestedSetValueChanged(ctrlType ctrl, double setValue, bool forwardToGui, bool isLeftcircle = false);
	void sigCircleDirectionChanged(bool isLeftCircle, double radius);

	void sigDisplayFlightPhase(QString flightPhase, QString toGo);
	void sigPathTrackingStatus(bool isTracking);

private:

	static DataCenter *dc;
	void clearOutSchedule(void);
	void clearOutStats(void);
	json dubinsPathCharacteristicsAsJson(void);
	json dubinsPathCartesiansAsJson(void);
	json statsSummaryAsJson(void);

	std::vector<FlightPhase*> flightPhases;
	unsigned int currentPhase = 0;
	const DubinsPath *db = NULL;
	Position_Cartesian initialDisplacement = Position_Cartesian();
	QTimer *flyPathTimer = NULL;
	const int timerMilliseconds = 100;

	//statistics
	class PathFollowStats {
	public:
		double cartesianPathLengthEarthFrame = 0.0, cartesianPathLengthWindFrame =0.0;
		double flightTime = 0.0;
		double heightLoss = 0.0;
		Position_Cartesian totalWindDisplacement = Position_Cartesian();
		double totalAngleFlown = 0.0;
		double glideAngleEarthFrame = 0.0;
		segmentTypeEnum segType = segmentTypeEnum::INVALID;
	};
	std::vector<PathFollowStats*> pathFollowStats;

};

#endif /* DUBINSSCHEDULER_H_ */
