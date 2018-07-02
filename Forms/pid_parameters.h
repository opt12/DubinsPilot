/*
 * pid_parameters.h
 *
 *  Created on: 25.04.2018
 *      Author: eckstein
 */

#ifndef FORMS_PID_PARAMETERS_H_
#define FORMS_PID_PARAMETERS_H_

#include <QDialog>

#include "ui_pid_parameters.h"
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_magnifier.h>
#include <qwt_plot_panner.h>
#include "enumDeclarations.h"
#include "Position.h"

class SpeedPlotData;


class PIDParametersDialog: public QDialog, public Ui::PIDParametersDialog {
Q_OBJECT

public:
	PIDParametersDialog(QWidget *parent = 0);

signals:
	void sigPidParametersChanged(ctrlType ctrl, double P, double I, double D);
	void sigCtrlActiveStateChanged(ctrlType ctrl, bool active);
	void sigRequestedSetValueChanged(ctrlType ctrl, double setValue);

	void sigSendXPDataRef(const char*, double);

	void sigLoggingActiveStateChanged(bool active, QFile* fileLog);
	void resetOrigin(void);
	void sigSetElfLocation(double forward, double right, double height, double rotation);

public slots:
	void setXPlaneConnection(bool);
	void attachControllerCurve(ctrlType c, QwtPlotCurve* ctrlCurve);
	void replotControllerCurve(ctrlType c);
	void showOriginCoords(Position_WGS84 origin);
	void showElfCoords(Position_WGS84 elf, Position_Cartesian elf_Cart, double elfHeading);
	void setRollControlKnob(double roll);	//TODO anschliessen
	void setClimbRateControlKnob(double glideAngle);	//TODO anschliessen

private slots:
	void setDValue(void);
	void setPValue(void);
	void setIValue(void);

	void setDSpinners(QString text);
	void setPSpinners(QString text);
	void setISpinners(QString text);
	void setClimbRateLabel(double val);

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

	void tempomatActiveStateChanged(bool active);
	void bankControlActiveStateChanged(bool active);
	void headingControlActiveStateChanged(bool active);
	void circleControlActiveStateChanged(bool active);

	void writeSettings();

	void setAltitude(void);

	void logButtonClicked(void);
	void fileNameSelectorClicked(void);
	void fileNameEditingFinished(void);

	void clearInputFields(void);
	void submitElfData(void);

protected:
	void closeEvent(QCloseEvent *event);

private:
	bool allowSave() {
		return false;
	} //maybe I don't want to always save the parameters at closing
	void readSettings();
	double valP, valI, valD, valClimbRate;
	double valPBank, valIBank, valDBank, valRoll;
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

};

#endif /* FORMS_PID_PARAMETERS_H_ */
