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
#include "enumDeclarations.h"
#include "FlightPhase.h"
#include "DubinsPath.h"


class DubinsScheduler : public QObject {

	Q_OBJECT

public:
	DubinsScheduler(QObject* parent = 0);
	virtual ~DubinsScheduler();

public slots:
	void takeMeDown(bool active){
		if(active){
		// TODO das wäre deutlich besser, hier einfach die Zielpositiion anzugeben und den Pfadtyp
		// TODO und dann halt selber rechnen. Dann kann der Dubins Pfad sogar lokal bleiben
		initialize(DataCenter::getInstance()->getCurrentDubinsPath());	// XXX
		flyPathTimer->start(timerMilliseconds);
		} else {
			flyPathTimer->stop();
		}
	}

private slots:
	void flyThePath(void);
	void initialize(const DubinsPath* _db);

signals:
	void sigCtrlActiveStateChanged(ctrlType ctrl, bool active);
	void sigRequestedSetValueChanged(ctrlType ctrl, double setValue);
	void sigCircleDirectionChanged(bool isLeftCircle, double radius);

private:

	void clearOutSchedule(void);

	std::vector<FlightPhase*> flightPhases;
	const DubinsPath *db = NULL;
	QTimer *flyPathTimer = NULL;
	const int timerMilliseconds = 100;
};

#endif /* DUBINSSCHEDULER_H_ */
