/*
 * pid_parameters.h
 *
 *  Created on: 25.04.2018
 *      Author: eckstein
 */

#ifndef FORMS_PID_PARAMETERS_H_
#define FORMS_PID_PARAMETERS_H_

#include <QDialog>
#include <QDir>

#include "ui_DubinsPilot_Dialog.h"
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_magnifier.h>
#include <qwt_plot_panner.h>
#include "enumDeclarations.h"
#include "Position.h"

class SpeedPlotData;


class DubinsPilotDialog: public QDialog, public Ui::DubinsPilotDialog {
Q_OBJECT

public:
	DubinsPilotDialog(QWidget *parent = 0);

signals:
	void sigPidParametersChanged(ctrlType ctrl, double P, double I, double D);
	void sigCtrlActiveStateChanged(ctrlType ctrl, bool active);
	void sigRequestedSetValueChanged(ctrlType ctrl, double setValue);
	void sigCircleDirectionChanged(bool isLeftCircle, double radius);
	void sigFlightPathCharacteristicsChanged(double valClimbRateStraight,
			double valClimbRateCircle, double valAutoCircleRadius);
	void sigApproachStartingAltitudeChanged(QString approachStartingAltitude);
	void sigLifeSaverHeightChanged(QString lifeSaverHeight);


	void sigSendXPDataRef(const char*, double);

	void resetOrigin(void);
	void sigSetElfLocation(double forward, double right, double height, double rotation, pathTypeEnum pathType);
	void sigSetElfLocation(Position_WGS84 elfPosition, double heading, pathTypeEnum pathType);
	void sigResetElf(void);
	void sigSetSimulationPaused(bool isPaused);

	void sigStartPathTracking(bool active);

	void sigLoggingActiveStateChanged(bool isLoggingActive, QDir dir, QString name);
	void sigLoggingActiveStateChanged(bool isLoggingActive);

public slots:
	void setXPlaneConnection(bool);
	void attachControllerCurve(ctrlType c, QwtPlotCurve* ctrlCurve);
	void replotControllerCurve(ctrlType c);
//	void showOriginCoords(Position_WGS84 origin);
	void showElfCoords(Position_WGS84 elf, double elfHeading, bool pathFeasible);
	void setTargetValueControlKnob(ctrlType ctrl, double targetValue, bool isLeftCircle=true);
	void setControllerCheckButtons(ctrlType ctrl, bool enable, bool checked);
	void displayCurrentWind(double windDirFrom, double windVelocity);
	void displayFlightPhase(QString flightPhase, QString toGo);
	void displayPathTrackingStatus(bool _isPathTracking);

	void clickSetHeight(void);	//to enable the "Life Saver" when Altitude AGL is too low
	void clickTakeMeDown(void); //to start path tracking at a defined altitude
	void clickPause(bool isPaused);

	void setLoggingState(bool _loggingActive);

private slots:
	void setDValue(void);
	void setPValue(void);
	void setIValue(void);

	void setDSpinners(QString text);
	void setPSpinners(QString text);
	void setISpinners(QString text);
	void setClimbRateLabel(double val);
	void setClimbRateStraightLabel(double val);
	void setClimbRateCircleLabel(double val);
	void setAutoCircleRadiusLabel(double radius);


	void setDValueBank(void);
	void setPValueBank(void);
	void setIValueBank(void);

	void setDSpinnersBank(QString text);
	void setPSpinnersBank(QString text);
	void setISpinnersBank(QString text);
	void setBankingLabel(double val);

	void setDValueHeading(void);
	void setPValueHeading(void);
	void setIValueHeading(void);

	void setDSpinnersHeading(QString text);
	void setPSpinnersHeading(QString text);
	void setISpinnersHeading(QString text);
	void setHeadingLabel(double val);

	void setDValueCircle(void);
	void setPValueCircle(void);
	void setIValueCircle(void);

	void setDSpinnersCircle(QString text);
	void setPSpinnersCircle(QString text);
	void setISpinnersCircle(QString text);
	void setCircleLabel(double val);

	void readSettings();	//needs to be a slot to use single shot timer

	void tempomatActiveStateChanged(bool active);
	void rollControlActiveStateChanged(bool active);
	void headingControlActiveStateChanged(bool active);
	void circleControlActiveStateChanged(bool active);
	void radioButtonCircleClicked(void);
	void continuousDubinsCalc(bool active);
	void contCalcTimerExpired(void);

	void writeSettings();

	void setAltitude(void);
	void setIgniterOff(void);

	void logButtonClicked(void);
	void fileNameSelectorClicked(void);
	void fileNameEditingFinished(void);

	void clearInputFields(void);
	void elfInputFieldsetDirty(void) { elfInputFieldsDirty = true; }
	void submitElfData(void);

	void takeMeDown(void);

	void setWind(void);

	void pauseSimulation(bool pause);

	void toggleAbsoluteRelative(void);

protected:
	void closeEvent(QCloseEvent *event);
	void reject();


private:
	bool allowSave() {
		return false;
	} //maybe I don't want to always save the parameters at closing
	double valPClimb, valIClimb, valDClimb, valClimbRate;
	double valClimbRateStraight, valClimbRateCircle, valAutoCircleRadius;
	double valPRoll, valIRoll, valDRoll, valRoll;
	double valPHeading, valIHeading, valDHeading, valHeading;
	double valPCircle, valICircle, valDCircle, valCircle;

	QwtPlotMarker *climbMarker, *rollMarker, *headingMarker, *circleMarker;
	QwtPlotMagnifier *zoom_yLeftClimb, *zoom_yRightClimb,
			*zoom_yLeftRoll, *zoom_yRightRoll,
			*zoom_yLeftHeading, *zoom_yRightHeading,
			*zoom_yLeftCircle, *zoom_yRightCircle;
	QwtPlotPanner *drag_yLeftClimb, *drag_yRightClimb,
			*drag_yLeftRoll, *drag_yRightRoll,
			*drag_yLeftHeading, *drag_yRightHeading,
			*drag_yLeftCircle, *drag_yRightCircle;

	void setupPlot(void);
	static const int plotDataSize = 500;
	static const int minExp = -6;
	static const int maxExp = 3;

	QString initialLogFileDir;
	QString logFileName, logFile;
	QString generateLogfilename();
	bool loggingActive = false;
	QFile fileLog;
	QTextStream *outLog;
	bool elfInputFieldsDirty = true;
	Position_WGS84 elfPosition;
	double elfHeading = 0.0;
	bool isElfSet = false;
	QTimer *contCalcTimer = NULL;
	const int timerMilliseconds = 500;
	bool isPathTracking = false;

};

#endif /* FORMS_PID_PARAMETERS_H_ */
