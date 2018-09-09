/*
 * ControlAutomation.h
 *
 *  Created on: 01.09.2018
 *      Author: eckstein
 */

#ifndef CONTROLAUTOMATION_H_
#define CONTROLAUTOMATION_H_

#include <QObject>
#include <QTimer>
#include"utils.h"

class DataCenter;

class ControlAutomation : public QObject {

	Q_OBJECT

public:
	ControlAutomation(QObject* parent = 0);
	virtual ~ControlAutomation();

public slots:
	void setLifeSaverHeightChanged(QString lifeSaverHeight);
	void setApproachStartingAltitudeChanged(QString approachStartingAltitude);

private slots:
	void checkForAutomationEvents(void);

signals:
	void sigLifeSaverTriggered(void);
	void sigApproachStartingTriggered(void);

private:
	static const double MAX;
	static const int TIMER_VALUE;

	bool debug = false;

	double approachStartAltitude = 0.0;
	double lifeSaverHeight= ft_to_m(50.0);
	bool approachStartArmed = false;
	bool lifeSaverArmed = true;
	DataCenter* dc;
	QTimer* automationTimer;
};

#endif /* CONTROLAUTOMATION_H_ */
