/*
 * pid_parameters.cpp
 *
 *  Created on: 25.04.2018
 *      Author: eckstein
 */

#include <QtGui>

#include "pid_parameters.h"
#include <qwt_plot.h>
#include <QPen>
#include <qwt_plot_curve.h>
#include <qwt_series_data.h>
#include <qwt_magnifier.h>
#include <qwt_plot_magnifier.h>
#include "Dataset.h"

PIDParametersDialog::PIDParametersDialog(QWidget* parent) :
		QDialog(parent) {

	setupUi(this);

	doubleSpinBoxDValue->setRange(-10.0, 10.0);
	doubleSpinBoxDValue->setSingleStep(0.01);
	SliderDValue->setRange(-10.0, 10.0, 0.1);
	spinBoxExpDValue->setRange(minExp, maxExp);
	connect(doubleSpinBoxDValue, SIGNAL(valueChanged(double)), this,
			SLOT(setDValue(void)));
	connect(spinBoxExpDValue, SIGNAL(valueChanged(int)), this,
			SLOT(setDValue(void)));
	lineEditDParameter->setValidator(new QDoubleValidator);
	connect(lineEditDParameter, SIGNAL(textChanged(QString)), this,
			SLOT(setDSpinners(QString)));

	doubleSpinBoxIValue->setMinimum(-10);
	doubleSpinBoxIValue->setMaximum(10);
	doubleSpinBoxIValue->setSingleStep(0.1);
	SliderIValue->setRange(-10.0, 10.0, 0.1);
	spinBoxExpIValue->setRange(minExp, maxExp);
	connect(doubleSpinBoxIValue, SIGNAL(valueChanged(double)), this,
			SLOT(setIValue(void)));
	connect(spinBoxExpIValue, SIGNAL(valueChanged(int)), this,
			SLOT(setIValue(void)));
	lineEditIParameter->setValidator(new QDoubleValidator);
	connect(lineEditIParameter, SIGNAL(textChanged(QString)), this,
			SLOT(setISpinners(QString)));

	doubleSpinBoxPValue->setMinimum(-10);
	doubleSpinBoxPValue->setMaximum(10);
	doubleSpinBoxPValue->setSingleStep(0.1);
	SliderPValue->setRange(-10.0, 10.0, 0.1);
	spinBoxExpPValue->setRange(minExp, maxExp);
	connect(doubleSpinBoxPValue, SIGNAL(valueChanged(double)), this,
			SLOT(setPValue(void)));
	connect(spinBoxExpPValue, SIGNAL(valueChanged(int)), this,
			SLOT(setPValue(void)));
	lineEditPParameter->setValidator(new QDoubleValidator);
	connect(lineEditPParameter, SIGNAL(textChanged(QString)), this,
			SLOT(setPSpinners(QString)));

	doubleSpinBoxDValueBank->setRange(-10.0, 10.0);
	doubleSpinBoxDValueBank->setSingleStep(0.01);
	SliderDValueBank->setRange(-10.0, 10.0, 0.1);
	spinBoxExpDValueBank->setRange(minExp, maxExp);
	connect(doubleSpinBoxDValueBank, SIGNAL(valueChanged(double)), this,
			SLOT(setDValueBank(void)));
	connect(spinBoxExpDValueBank, SIGNAL(valueChanged(int)), this,
			SLOT(setDValueBank(void)));
	lineEditDParameterBank->setValidator(new QDoubleValidator);
	connect(lineEditDParameterBank, SIGNAL(textChanged(QString)), this,
			SLOT(setDSpinnersBank(QString)));

	doubleSpinBoxIValueBank->setMinimum(-10);
	doubleSpinBoxIValueBank->setMaximum(10);
	doubleSpinBoxIValueBank->setSingleStep(0.1);
	SliderIValueBank->setRange(-10.0, 10.0, 0.1);
	spinBoxExpIValueBank->setRange(minExp, maxExp);
	connect(doubleSpinBoxIValueBank, SIGNAL(valueChanged(double)), this,
			SLOT(setIValueBank(void)));
	connect(spinBoxExpIValueBank, SIGNAL(valueChanged(int)), this,
			SLOT(setIValueBank(void)));
	lineEditIParameterBank->setValidator(new QDoubleValidator);
	connect(lineEditIParameterBank, SIGNAL(textChanged(QString)), this,
			SLOT(setISpinnersBank(QString)));

	doubleSpinBoxPValueBank->setMinimum(-10);
	doubleSpinBoxPValueBank->setMaximum(10);
	doubleSpinBoxPValueBank->setSingleStep(0.1);
	SliderPValueBank->setRange(-10.0, 10.0, 0.1);
	spinBoxExpPValueBank->setRange(minExp, maxExp);
	connect(doubleSpinBoxPValueBank, SIGNAL(valueChanged(double)), this,
			SLOT(setPValueBank(void)));
	connect(spinBoxExpPValueBank, SIGNAL(valueChanged(int)), this,
			SLOT(setPValueBank(void)));
	lineEditPParameterBank->setValidator(new QDoubleValidator);
	connect(lineEditPParameterBank, SIGNAL(textChanged(QString)), this,
			SLOT(setPSpinnersBank(QString)));

	connect(saveButton, SIGNAL(clicked(void)), this, SLOT(writeSettings(void)));
	connect(setHeightButton, SIGNAL(clicked(void)), this,
			SLOT(setAltitude(void)));

	setupPlot();

//	KnobSpeed->setRange(-8.0, 8.0, .1);	//for Pitch-Control
	KnobSpeed->setRange(-15, 15, 0.025);
	connect(KnobSpeed, SIGNAL(valueChanged(double)), this,
			SLOT(setClimbRateLabel(double)));

	KnobBank->setRange(-30.0, 30.0, 1.0);
	connect(KnobBank, SIGNAL(valueChanged(double)), this,
			SLOT(setBankingLabel(double)));

	checkBoxPID->setChecked(false);
	connect(checkBoxPID, SIGNAL(toggled(bool)), this,
			SLOT(tempomatActiveStateChanged(bool)));
	ledIndicator->setChecked(false);

	checkBoxPidBank->setChecked(false);

	connect(checkBoxPidBank, SIGNAL(toggled(bool)), this,
			SLOT(bankControlActiveStateChanged(bool)));
	connect(checkBoxPidBank, SIGNAL(toggled(bool)), this,
			SIGNAL(sigCheckboxBankingChanged(bool)));
	ledIndicatorBank->setChecked(false);

	//logging
	connect(toggleLogButton, SIGNAL(clicked()), this, SLOT(logButtonClicked()));
	connect(choseFileNameButton, SIGNAL(clicked()), this,
			SLOT(fileNameSelectorClicked()));
	connect(lineEditFileName, SIGNAL(editingFinished()), this,
			SLOT(fileNameEditingFinished()));
	toggleLogButton->setEnabled(false);


	//do this last to have everything wired together already
	readSettings();

	if (initialLogFileDir.isEmpty())
		initialLogFileDir = QDir::homePath();
	logFileName = generateLogfilename();
	logFile = QDir(initialLogFileDir).filePath(logFileName);
	lineEditFileName->setText(logFile);

	//TODO
	checkBoxPID->setEnabled(true);
	checkBoxPidBank->setEnabled(true);

}

//Implementation of protected functions
void PIDParametersDialog::closeEvent(QCloseEvent *event) {
	if (allowSave()) {
		writeSettings();
		event->accept();
	} else {
		event->accept();	//indeed, we want to close
//		event->ignore();
	}
}

void PIDParametersDialog::setXPlaneConnection(bool active) {
	qDebug() << "XPlane Connection changed" << endl;
	ledIndicator->setChecked(active);
	ledIndicatorBank->setChecked(active);
	checkBoxPID->setEnabled(active);
	checkBoxPidBank->setEnabled(active);
	if (!active)
		checkBoxPID->setChecked(false);
	if (!active)
		checkBoxPidBank->setChecked(false);
	if(loggingActive){
		logButtonClicked();	//close the logfile;
	}
	toggleLogButton->setEnabled(active);
}

void PIDParametersDialog::attachSpeedCurve(QwtPlotCurve* speedCurve) {
	qDebug() << "Speed curve attached\n";
	speedCurve->attach(qwtPlotSpeed);
}

void PIDParametersDialog::replotSpeedCurve(void) {
	qwtPlotSpeed->replot();
}

void PIDParametersDialog::attachRollCurve(QwtPlotCurve* rollCurve) {
	qDebug() << "Roll curve attached\n";
	rollCurve->attach(qwtPlotRoll);
}

void PIDParametersDialog::replotRollCurve(void) {
	qwtPlotRoll->replot();
}

void PIDParametersDialog::setDValue(void) {
	double mantissa = doubleSpinBoxDValue->value();
	int exp = spinBoxExpDValue->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditDParameter->setText(QString::number(value, 'g'));
	valD = value;
	emit sigPidParametersChanged(valP, valI, valD);
}
void PIDParametersDialog::setIValue(void) {
	double mantissa = doubleSpinBoxIValue->value();
	int exp = spinBoxExpIValue->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditIParameter->setText(QString::number(value, 'g'));
	valI = value;
	emit sigPidParametersChanged(valP, valI, valD);
}
void PIDParametersDialog::setPValue(void) {
	double mantissa = doubleSpinBoxPValue->value();
	int exp = spinBoxExpPValue->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditPParameter->setText(QString::number(value, 'g'));
	valP = value;
	emit sigPidParametersChanged(valP, valI, valD);
}

void PIDParametersDialog::setDSpinners(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxDValue->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
//	qDebug() << "D value changed to: " << value << "; this is " << mantissa
//			<< "E" << exp << endl;
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxDValue->setValue(mantissa);
		spinBoxExpDValue->setValue(exp);
		valD = value;
	}
}

void PIDParametersDialog::setPSpinners(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxPValue->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
//	qDebug() << "P value changed to: " << value << "; this is " << mantissa
//			<< "E" << exp << endl;
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxPValue->setValue(mantissa);
		spinBoxExpPValue->setValue(exp);
		valP = value;
	}
}

void PIDParametersDialog::setISpinners(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxIValue->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
//	qDebug() << "I value changed to: " << value << "; this is " << mantissa
//			<< "E" << exp << endl;
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxIValue->setValue(mantissa);
		spinBoxExpIValue->setValue(exp);
		valI = value;
	}
}

void PIDParametersDialog::setClimbRateLabel(double climbRate) {
	valClimbRate = climbRate;
	climbRateLabel->setText(
			"climb: " + QString::number(climbRate) + " [m/s]\n"
					+ QString::number(climbRate / 5.08e-3) + " [ft/min]");
	emit sigRequestedSpeedChanged(valClimbRate);
	speedMarker->setYValue(valClimbRate);
	speedMarker->setLabel("y = " + QString::number(valClimbRate));
	qwtPlotSpeed->replot();
}

void PIDParametersDialog::setDValueBank(void) {
	double mantissa = doubleSpinBoxDValueBank->value();
	int exp = spinBoxExpDValueBank->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditDParameterBank->setText(QString::number(value, 'g'));
	valDBank = value;
	emit sigPidParametersBankChanged(valPBank, valIBank, valDBank);
}
void PIDParametersDialog::setIValueBank(void) {
	double mantissa = doubleSpinBoxIValueBank->value();
	int exp = spinBoxExpIValueBank->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditIParameterBank->setText(QString::number(value, 'g'));
	valIBank = value;
	emit sigPidParametersBankChanged(valPBank, valIBank, valDBank);
}
void PIDParametersDialog::setPValueBank(void) {
	double mantissa = doubleSpinBoxPValueBank->value();
	int exp = spinBoxExpPValueBank->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditPParameterBank->setText(QString::number(value, 'g'));
	valPBank = value;
	emit sigPidParametersBankChanged(valPBank, valIBank, valDBank);
}

void PIDParametersDialog::setDSpinnersBank(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxDValueBank->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxDValueBank->setValue(mantissa);
		spinBoxExpDValueBank->setValue(exp);
		valDBank = value;
	}
}

void PIDParametersDialog::setPSpinnersBank(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxPValueBank->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxPValueBank->setValue(mantissa);
		spinBoxExpPValueBank->setValue(exp);
		valP = value;
	}
}

void PIDParametersDialog::setISpinnersBank(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxIValueBank->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxIValueBank->setValue(mantissa);
		spinBoxExpIValueBank->setValue(exp);
		valIBank = value;
	}
}

void PIDParametersDialog::setBankingLabel(double banking) {
	valBanking = banking;
	bankLabel->setText("Roll: " + QString::number(valBanking) + " [deg]");
	emit sigRequestedBankingChanged(banking);
	rollMarker->setYValue(valBanking);
	rollMarker->setLabel("roll = " + QString::number(valBanking));
	qwtPlotRoll->replot();

}

void PIDParametersDialog::tempomatActiveStateChanged(bool active) {
	emit sigPidParametersChanged(valP, valI, valD);
	emit sigRequestedSpeedChanged(valClimbRate);
	emit sigTempomatActiveStateChanged(active);
}

void PIDParametersDialog::bankControlActiveStateChanged(bool active) {
	emit sigBankControlActiveStateChanged(active);
	emit sigRequestedBankingChanged(valBanking);
	emit sigPidParametersBankChanged(valPBank, valIBank, valDBank);
}

void PIDParametersDialog::readSettings() {
	QSettings settings("EEE", "PID_Parameters");

	qDebug() << "reading settings from " << settings.fileName() << endl;

	restoreGeometry(settings.value("geometry").toByteArray());

	valP = settings.value("valP").toDouble();
	qDebug() << "valP: " << valP;
	lineEditPParameter->setText(QString::number(valP, 'g'));
	valI = settings.value("valI").toDouble();
	qDebug() << "\tvalI: " << valI;
	lineEditIParameter->setText(QString::number(valI, 'g'));
	valD = settings.value("valD").toDouble();
	qDebug() << "\tvalD: " << valD;
	lineEditDParameter->setText(QString::number(valD, 'g'));
	valClimbRate = settings.value("valClimbRate").toDouble();
	KnobSpeed->setValue(valClimbRate);
	qDebug() << "\tvalSpeed: " << valClimbRate << endl;
	checkBoxPID->setChecked(settings.value("tempomatActive").toBool());

	valPBank = settings.value("valPBank").toDouble();
	qDebug() << "valPBank: " << valPBank;
	lineEditPParameterBank->setText(QString::number(valPBank, 'g'));
	valIBank = settings.value("valIBank").toDouble();
	qDebug() << "\tvalIBank: " << valIBank;
	lineEditIParameterBank->setText(QString::number(valIBank, 'g'));
	valDBank = settings.value("valDBank").toDouble();
	qDebug() << "\tvalDBank: " << valDBank;
	lineEditDParameterBank->setText(QString::number(valDBank, 'g'));
	valBanking = settings.value("valBanking").toDouble();
	KnobBank->setValue(valBanking);
	qDebug() << "\tvalBanking: " << valBanking << endl;
	checkBoxPidBank->setChecked(
			settings.value("bankingControlActive").toBool());

	initialLogFileDir = settings.value("initialLogFileDir").toString();
}

void PIDParametersDialog::writeSettings() {
	QSettings settings("EEE", "PID_Parameters");
	qDebug() << "Saving: valP: " << valP;
	qDebug() << "\tvalI: " << valI;
	qDebug() << "\tvalD: " << valD;
	qDebug() << "\tvalSpeed: " << valClimbRate << endl;

	settings.setValue("valP", QVariant::fromValue(valP));
	settings.setValue("valI", QVariant::fromValue(valI));
	settings.setValue("valD", QVariant::fromValue(valD));
	settings.setValue("valClimbRate", QVariant::fromValue(valClimbRate));

	settings.setValue("valPBank", QVariant::fromValue(valPBank));
	settings.setValue("valIBank", QVariant::fromValue(valIBank));
	settings.setValue("valDBank", QVariant::fromValue(valDBank));
	settings.setValue("valBanking", QVariant::fromValue(valBanking));

	settings.setValue("initialLogFileDir",
			QVariant::fromValue(initialLogFileDir));
}

void PIDParametersDialog::setAltitude(void) {
	double altitudeAboveGround = lineEditAltitude->text().toDouble();
	emit sigSetRequestedAltitude(altitudeAboveGround);
}

void PIDParametersDialog::logButtonClicked() {
	qDebug() << "logButtonClicked(), loggingActive(before) = " << loggingActive << endl;
	if (loggingActive) {
		loggingActive = false;
		emit sigLoggingActiveStateChanged(loggingActive, &fileLog);
		if(fileLog.isOpen())
			fileLog.close();
		free(outLog);
		//stop logging
		lineEditFileName->setEnabled(true);
		choseFileNameButton->setEnabled(true);
		toggleLogButton->setText(tr("Start &Logging"));
	} else {
		//start logging
		QFileInfo fileName = QFileInfo(initialLogFileDir, generateLogfilename());
		fileLog.setFileName(fileName.absoluteFilePath());
		if(!fileLog.open(QIODevice::WriteOnly)){
			qDebug() <<"Cannot open file for writing: "
					<<qPrintable(fileLog.errorString())<<endl;
			return;
		}
		lineEditFileName->setText(fileName.absoluteFilePath());
		lineEditFileName->setEnabled(false);
		choseFileNameButton->setEnabled(false);
		outLog = new QTextStream(&fileLog);
		qDebug() << Dataset::csvHeading();
		*outLog << Dataset::csvHeading();
		outLog->flush();
		toggleLogButton->setText(tr("Stop &Logging"));
		loggingActive = true;
		emit sigLoggingActiveStateChanged(loggingActive, &fileLog);
	}
}
void PIDParametersDialog::fileNameSelectorClicked(void) {
	initialLogFileDir = QFileDialog::getExistingDirectory(this, tr("Open Directory for logging:"),
			initialLogFileDir,
			QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	logFile = generateLogfilename();
	lineEditFileName->setText(QFileInfo(initialLogFileDir, logFile).absoluteFilePath());
}

void PIDParametersDialog::fileNameEditingFinished() {
	QFileInfo cleanPath = QFileInfo(lineEditFileName->text());
	qDebug() << "Set logfile to: " << cleanPath.absoluteFilePath() << endl;
	initialLogFileDir = cleanPath.filePath();
	logFile = cleanPath.fileName();
	lineEditFileName->setText(cleanPath.absoluteFilePath());
}

QString PIDParametersDialog::generateLogfilename() {
	QString DateString =
			QDateTime::currentDateTime().toString(Qt::ISODate).replace('-', '_').replace(
					':', '_');
	return "Log_" + DateString + ".csv";
}

void PIDParametersDialog::timerEvent(QTimerEvent *) {
//	double inSpeedVal = 20 * sin( M_PI * count / 50)+valSpeed;
//	double inElevatorVal = 30 * cos( M_PI * count / 50)+valSpeed;
//	++count;

//	qDebug()<<"Timer event ticked";

// add the new input to the plot
//	memmove(speedData, speedData + 1, (plotDataSize - 1) * sizeof(double));
//	memmove(elevatorData, elevatorData + 1,
//			(plotDataSize - 1) * sizeof(double));
//	speedData[plotDataSize - 1] = inSpeedVal;
//	elevatorData[plotDataSize - 1] = inElevatorVal;
//	speedCurve->setSamples(timeData, speedData, plotDataSize);
//	elevatorCurve->setSamples(timeData, elevatorData, plotDataSize);
//
//	qwtPlotSpeed->replot();
}

void PIDParametersDialog::setupPlot(void) {
	qwtPlotSpeed->setTitle("Sink Rate");

// axes
	qwtPlotSpeed->setAxisTitle(qwtPlotSpeed->xBottom, "t -->");
	qwtPlotSpeed->setAxisScale(qwtPlotSpeed->xBottom, 0.0, plotDataSize);

	qwtPlotSpeed->setAxisTitle(qwtPlotSpeed->yLeft, "climb Rate[m/s] -->");
	qwtPlotSpeed->setAxisScale(qwtPlotSpeed->yLeft, -10, 10);

	qwtPlotSpeed->setAxisTitle(qwtPlotSpeed->yRight, "<-- elevator");
	qwtPlotSpeed->setAxisScale(qwtPlotSpeed->yRight, -0.75, 0.75);

	qwtPlotSpeed->enableAxis(QwtPlot::xBottom);
	qwtPlotSpeed->enableAxis(QwtPlot::yLeft);
	qwtPlotSpeed->enableAxis(QwtPlot::yRight);

//  ...a horizontal line at y = 0...
	speedMarker = new QwtPlotMarker();
	speedMarker->setLabel(QString::fromLatin1("y = 60"));
	speedMarker->setLabelAlignment(Qt::AlignRight | Qt::AlignTop);
	speedMarker->setLineStyle(QwtPlotMarker::HLine);
//	speedMarker->setYValue(0.0);
	speedMarker->attach(qwtPlotSpeed);

	zoom_yLeftClimb = new QwtPlotMagnifier(qwtPlotSpeed->canvas());
	zoom_yLeftClimb->setWheelButtonState(Qt::NoModifier);
	zoom_yLeftClimb->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yLeftClimb->setAxisEnabled(QwtPlot::yLeft, true);
	zoom_yLeftClimb->setAxisEnabled(QwtPlot::yRight, false);

	drag_yLeftClimb = new QwtPlotPanner(qwtPlotSpeed->canvas());
	drag_yLeftClimb->setMouseButton(Qt::LeftButton, Qt::NoModifier);
	drag_yLeftClimb->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yLeftClimb->setAxisEnabled(QwtPlot::yLeft, true);
	drag_yLeftClimb->setAxisEnabled(QwtPlot::yRight, false);

	zoom_yRightClimb = new QwtPlotMagnifier(qwtPlotSpeed->canvas());
	zoom_yRightClimb->setWheelButtonState(Qt::ControlModifier);
	zoom_yRightClimb->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yRightClimb->setAxisEnabled(QwtPlot::yLeft, false);
	zoom_yRightClimb->setAxisEnabled(QwtPlot::yRight, true);

	drag_yRightClimb = new QwtPlotPanner(qwtPlotSpeed->canvas());
	drag_yRightClimb->setMouseButton(Qt::LeftButton, Qt::ControlModifier);
	drag_yRightClimb->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yRightClimb->setAxisEnabled(QwtPlot::yLeft, false);
	drag_yRightClimb->setAxisEnabled(QwtPlot::yRight, true);

	qwtPlotSpeed->replot();
	qwtPlotSpeed->show();

	qwtPlotRoll->setTitle("Roll Angle");

// axes
	qwtPlotRoll->setAxisTitle(qwtPlotRoll->xBottom, "t -->");
	qwtPlotRoll->setAxisScale(qwtPlotRoll->xBottom, 0.0, plotDataSize);

	qwtPlotRoll->setAxisTitle(qwtPlotRoll->yLeft, "Roll angle [deg] -->");
	qwtPlotRoll->setAxisScale(qwtPlotRoll->yLeft, -30, 30);

	qwtPlotRoll->setAxisTitle(qwtPlotRoll->yRight, "<-- aileron");
	qwtPlotRoll->setAxisScale(qwtPlotRoll->yRight, -0.75, 0.75);

	qwtPlotRoll->enableAxis(QwtPlot::xBottom);
	qwtPlotRoll->enableAxis(QwtPlot::yLeft);
	qwtPlotRoll->enableAxis(QwtPlot::yRight);

//  ...a horizontal line at y = 0...
	rollMarker = new QwtPlotMarker();
	rollMarker->setLabel(QString::fromLatin1("y = 60"));
	rollMarker->setLabelAlignment(Qt::AlignRight | Qt::AlignTop);
	rollMarker->setLineStyle(QwtPlotMarker::HLine);
//	rollMarker->setYValue(0.0);
	rollMarker->attach(qwtPlotRoll);

	zoom_yLeftRoll = new QwtPlotMagnifier(qwtPlotRoll->canvas());
	zoom_yLeftRoll->setWheelButtonState(Qt::NoModifier);
	zoom_yLeftRoll->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yLeftRoll->setAxisEnabled(QwtPlot::yLeft, true);
	zoom_yLeftRoll->setAxisEnabled(QwtPlot::yRight, false);

	drag_yLeftRoll = new QwtPlotPanner(qwtPlotRoll->canvas());
	drag_yLeftRoll->setMouseButton(Qt::LeftButton, Qt::NoModifier);
	drag_yLeftRoll->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yLeftRoll->setAxisEnabled(QwtPlot::yLeft, true);
	drag_yLeftRoll->setAxisEnabled(QwtPlot::yRight, false);

	zoom_yRightRoll = new QwtPlotMagnifier(qwtPlotRoll->canvas());
	zoom_yRightRoll->setWheelButtonState(Qt::ControlModifier);
	zoom_yRightRoll->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yRightRoll->setAxisEnabled(QwtPlot::yLeft, false);
	zoom_yRightRoll->setAxisEnabled(QwtPlot::yRight, true);

	drag_yRightRoll = new QwtPlotPanner(qwtPlotRoll->canvas());
	drag_yRightRoll->setMouseButton(Qt::LeftButton, Qt::ControlModifier);
	drag_yRightRoll->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yRightRoll->setAxisEnabled(QwtPlot::yLeft, false);
	drag_yRightRoll->setAxisEnabled(QwtPlot::yRight, true);

	qwtPlotRoll->replot();
	qwtPlotRoll->show();
}
