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


public slots:
	void setXPlaneConnection(bool);
	void attachControllerCurve(ctrlType c, QwtPlotCurve* ctrlCurve);
	void replotControllerCurve(ctrlType c);


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

	void tempomatActiveStateChanged(bool);
	void bankControlActiveStateChanged(bool);

	void writeSettings();

	void setAltitude(void);

	void logButtonClicked(void);
	void fileNameSelectorClicked(void);
	void fileNameEditingFinished(void);

protected:
	void closeEvent(QCloseEvent *event);

private:
	bool allowSave() {
		return false;
	} //maybe I don't want to always save the parameters at closing
	void readSettings();
	double valP, valI, valD, valClimbRate;
	double valPBank, valIBank, valDBank, valRoll;

	QwtPlotMarker *climbMarker, *rollMarker;
	QwtPlotMagnifier *zoom_yLeftClimb, *zoom_yRightClimb, *zoom_yLeftRoll, *zoom_yRightRoll;
	QwtPlotPanner *drag_yLeftClimb, *drag_yRightClimb, *drag_yLeftRoll, *drag_yRightRoll;

	void setupPlot(void);
	static const int plotDataSize = 500;
	static const int minExp = -6;
	static const int maxExp = 3;

	// data arrays for the plot
	double speedData[plotDataSize];
	double elevatorData[plotDataSize];
	double timeData[plotDataSize];
	int count;

	QString initialLogFileDir;
	QString logFileName, logFile;
	QString generateLogfilename();
	bool loggingActive = false;
	QFile fileLog;
	QTextStream *outLog;

};

#endif /* FORMS_PID_PARAMETERS_H_ */
