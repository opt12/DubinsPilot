/*
 * pid_parameters.cpp
 *
 *  Created on: 25.04.2018
 *      Author: eckstein
 */

//#define GENERATE_FLIGHT_TEST_VALUES
//#define GENERATE_CIRCLE_TEST_VALUES

#include <QtGui>

#include "DubinsPilot_Dialog.h"
#include <qwt_plot.h>
#include <QPen>
#include <qwt_plot_curve.h>
#include <qwt_series_data.h>
#include <qwt_magnifier.h>
#include <qwt_plot_magnifier.h>
#include <qwt_knob.h>
#include <qwt_compass.h>
#include <qwt_compass_rose.h>
#include <qwt_dial_needle.h>
#include <math.h>       /* tan */
#include "Dataset.h"
#include "utils.h"
#ifdef GENERATE_FLIGHT_TEST_VALUES
#include "enumDeclarations.h"
#endif

#include "DataCenter.h"

DubinsPilotDialog::DubinsPilotDialog(QWidget* parent) :
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

	doubleSpinBoxDValueHeading->setRange(-10.0, 10.0);
	doubleSpinBoxDValueHeading->setSingleStep(0.01);
	SliderDValueHeading->setRange(-10.0, 10.0, 0.1);
	spinBoxExpDValueHeading->setRange(minExp, maxExp);
	connect(doubleSpinBoxDValueHeading, SIGNAL(valueChanged(double)), this,
			SLOT(setDValueHeading(void)));
	connect(spinBoxExpDValueHeading, SIGNAL(valueChanged(int)), this,
			SLOT(setDValueHeading(void)));
	lineEditDParameterHeading->setValidator(new QDoubleValidator);
	connect(lineEditDParameterHeading, SIGNAL(textChanged(QString)), this,
			SLOT(setDSpinnersHeading(QString)));

	doubleSpinBoxIValueHeading->setMinimum(-10);
	doubleSpinBoxIValueHeading->setMaximum(10);
	doubleSpinBoxIValueHeading->setSingleStep(0.1);
	SliderIValueHeading->setRange(-10.0, 10.0, 0.1);
	spinBoxExpIValueHeading->setRange(minExp, maxExp);
	connect(doubleSpinBoxIValueHeading, SIGNAL(valueChanged(double)), this,
			SLOT(setIValueHeading(void)));
	connect(spinBoxExpIValueHeading, SIGNAL(valueChanged(int)), this,
			SLOT(setIValueHeading(void)));
	lineEditIParameterHeading->setValidator(new QDoubleValidator);
	connect(lineEditIParameterHeading, SIGNAL(textChanged(QString)), this,
			SLOT(setISpinnersHeading(QString)));

	doubleSpinBoxPValueHeading->setMinimum(-10);
	doubleSpinBoxPValueHeading->setMaximum(10);
	doubleSpinBoxPValueHeading->setSingleStep(0.1);
	SliderPValueHeading->setRange(-10.0, 10.0, 0.1);
	spinBoxExpPValueHeading->setRange(minExp, maxExp);
	connect(doubleSpinBoxPValueHeading, SIGNAL(valueChanged(double)), this,
			SLOT(setPValueHeading(void)));
	connect(spinBoxExpPValueHeading, SIGNAL(valueChanged(int)), this,
			SLOT(setPValueHeading(void)));
	lineEditPParameterHeading->setValidator(new QDoubleValidator);
	connect(lineEditPParameterHeading, SIGNAL(textChanged(QString)), this,
			SLOT(setPSpinnersHeading(QString)));

	doubleSpinBoxDValueCircle->setRange(-10.0, 10.0);
	doubleSpinBoxDValueCircle->setSingleStep(0.01);
	SliderDValueCircle->setRange(-10.0, 10.0, 0.1);
	spinBoxExpDValueCircle->setRange(minExp, maxExp);
	connect(doubleSpinBoxDValueCircle, SIGNAL(valueChanged(double)), this,
			SLOT(setDValueCircle(void)));
	connect(spinBoxExpDValueCircle, SIGNAL(valueChanged(int)), this,
			SLOT(setDValueCircle(void)));
	lineEditDParameterCircle->setValidator(new QDoubleValidator);
	connect(lineEditDParameterCircle, SIGNAL(textChanged(QString)), this,
			SLOT(setDSpinnersCircle(QString)));

	doubleSpinBoxIValueCircle->setMinimum(-10);
	doubleSpinBoxIValueCircle->setMaximum(10);
	doubleSpinBoxIValueCircle->setSingleStep(0.1);
	SliderIValueCircle->setRange(-10.0, 10.0, 0.1);
	spinBoxExpIValueCircle->setRange(minExp, maxExp);
	connect(doubleSpinBoxIValueCircle, SIGNAL(valueChanged(double)), this,
			SLOT(setIValueCircle(void)));
	connect(spinBoxExpIValueCircle, SIGNAL(valueChanged(int)), this,
			SLOT(setIValueCircle(void)));
	lineEditIParameterCircle->setValidator(new QDoubleValidator);
	connect(lineEditIParameterCircle, SIGNAL(textChanged(QString)), this,
			SLOT(setISpinnersCircle(QString)));

	doubleSpinBoxPValueCircle->setMinimum(-10);
	doubleSpinBoxPValueCircle->setMaximum(10);
	doubleSpinBoxPValueCircle->setSingleStep(0.1);
	SliderPValueCircle->setRange(-10.0, 10.0, 0.1);
	spinBoxExpPValueCircle->setRange(minExp, maxExp);
	connect(doubleSpinBoxPValueCircle, SIGNAL(valueChanged(double)), this,
			SLOT(setPValueCircle(void)));
	connect(spinBoxExpPValueCircle, SIGNAL(valueChanged(int)), this,
			SLOT(setPValueCircle(void)));
	lineEditPParameterCircle->setValidator(new QDoubleValidator);
	connect(lineEditPParameterCircle, SIGNAL(textChanged(QString)), this,
			SLOT(setPSpinnersCircle(QString)));

	connect(saveButton, SIGNAL(clicked(void)), this, SLOT(writeSettings(void)));
	connect(setHeightButton, SIGNAL(clicked(void)), this,
			SLOT(setAltitude(void)));
	connect(igniterOffButton, SIGNAL(clicked(void)), this,
			SLOT(setIgniterOff(void)));


	setupPlot();

//	KnobSpeed->setRange(-8.0, 8.0, .1);	//for Pitch-Control
	KnobClimbCircle->setRange(-10, 0, 0.025);
	connect(KnobClimbCircle, SIGNAL(valueChanged(double)), this,
			SLOT(setClimbRateCircleLabel(double)));
	KnobClimbStraight->setRange(-10, 0, 0.025);
	connect(KnobClimbStraight, SIGNAL(valueChanged(double)), this,
			SLOT(setClimbRateStraightLabel(double)));

	KnobAutoCircleRadius->setRange(300, 1000, 10.0);
	connect(KnobAutoCircleRadius, SIGNAL(valueChanged(double)), this,
			SLOT(setAutoCircleRadiusLabel(double)));


	KnobClimb->setRange(-15, 15, 0.025);
	connect(KnobClimb, SIGNAL(valueChanged(double)), this,
			SLOT(setClimbRateLabel(double)));

	KnobRoll->setRange(-30.0, 30.0, 1.0);
	connect(KnobRoll, SIGNAL(valueChanged(double)), this,
			SLOT(setBankingLabel(double)));

	CompassHeading->setNeedle(
			new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, true, Qt::red));
	CompassHeading->setRose(new QwtSimpleCompassRose(4, 1));
	CompassHeading->setRange(0.0, 360.0, 5.0, 1);
	connect(CompassHeading, SIGNAL(valueChanged(double)), this,
			SLOT(setHeadingLabel(double)));

	KnobCircle->setRange(300, 1000, 10.0);
	connect(KnobCircle, SIGNAL(valueChanged(double)), this,
			SLOT(setCircleLabel(double)));

	checkBoxClimbController->setChecked(false);
	connect(checkBoxClimbController, SIGNAL(clicked(bool)), this,
			SLOT(tempomatActiveStateChanged(bool)));
	ledIndicator->setChecked(false);

	checkBoxRollController->setChecked(false);

	connect(checkBoxHeadingController, SIGNAL(clicked(bool)), this,
			SLOT(headingControlActiveStateChanged(bool)));
	ledIndicatorHeading->setChecked(false);

	checkBoxHeadingController->setChecked(false);

	connect(checkBoxCircleController, SIGNAL(clicked(bool)), this,
			SLOT(circleControlActiveStateChanged(bool)));
	ledIndicatorCircle->setChecked(false);

	checkBoxCircleController->setChecked(false);

	connect(checkBoxRollController, SIGNAL(clicked(bool)), this,
			SLOT(rollControlActiveStateChanged(bool)));
	ledIndicatorRoll->setChecked(false);

	//logging
	connect(toggleLogButton, SIGNAL(clicked()), this, SLOT(logButtonClicked()));
	connect(choseFileNameButton, SIGNAL(clicked()), this,
			SLOT(fileNameSelectorClicked()));
	connect(lineEditFileName, SIGNAL(editingFinished()), this,
			SLOT(fileNameEditingFinished()));
//	toggleLogButton->setEnabled(false);

	if (initialLogFileDir.isEmpty())
		initialLogFileDir = QDir::homePath();
	logFileName = generateLogfilename();
	logFile = QDir(initialLogFileDir).filePath(logFileName);
	lineEditFileName->setText(logFile);

	connect(pushButtonResetElf, SIGNAL(clicked()), this,
			SLOT(clearInputFields(void)));

	connect(radioButtonLSL, SIGNAL(clicked()), this,
			SLOT(submitElfData()));
	connect(radioButtonRSR, SIGNAL(clicked()), this,
			SLOT(submitElfData()));

	connect(radioButtonRelative, SIGNAL(clicked()), this,
				SLOT(toggleAbsoluteRelative()));
	connect(radioButtonAbsolute, SIGNAL(clicked()), this,
				SLOT(toggleAbsoluteRelative()));

	lineEditForward->setValidator(new QDoubleValidator);
	connect(lineEditForward, SIGNAL(textChanged(QString)), this,
			SLOT(elfInputFieldsetDirty(void)));
	lineEditRight->setValidator(new QDoubleValidator);
	connect(lineEditRight, SIGNAL(textChanged(QString)), this,
			SLOT(elfInputFieldsetDirty(void)));
	lineEditHeight->setValidator(new QDoubleValidator);
	connect(lineEditHeight, SIGNAL(textChanged(QString)), this,
			SLOT(elfInputFieldsetDirty(void)));
	lineEditRotation->setValidator(new QDoubleValidator);
	connect(lineEditRotation, SIGNAL(textChanged(QString)), this,
			SLOT(elfInputFieldsetDirty(void)));

	connect(pushButtonSubmitElf, SIGNAL(clicked()), this,
			SLOT(submitElfData(void)));

	connect(radioButtonCircleLeft, SIGNAL(clicked()), this,
			SLOT(radioButtonCircleClicked()));
	connect(radioButtonCircleRight, SIGNAL(clicked()), this,
			SLOT(radioButtonCircleClicked()));

	connect(checkBoxCont, SIGNAL(clicked(bool)), this,
			SLOT(continuousDubinsCalc(bool)));
	contCalcTimer = new QTimer();
	connect(contCalcTimer, SIGNAL(timeout()), this, SLOT(contCalcTimerExpired()));

	connect(pushButtonFlyPath, SIGNAL(clicked(bool)), this,
			SLOT(takeMeDown()));


	CompassWind->setNeedle(
			new QwtCompassWindArrow(QwtCompassWindArrow::Style2, Qt::darkBlue,
					Qt::red));
	CompassWind->setRose(new QwtSimpleCompassRose(4, 1));
	CompassWind->setRange(0.0, 360.0, 5.0, 1);
	connect(CompassWind, SIGNAL(valueChanged(double)), this, SLOT(setWind()));

	SliderWindVelocity->setRange(0.0, ms_to_knots(45.0), ms_to_knots(0.25));
	connect(SliderWindVelocity, SIGNAL(valueChanged(double)), this,
			SLOT(setWind()));

	connect(checkBoxPause, SIGNAL(toggled(bool)), this, SLOT(pauseSimulation(bool)));

	connect(lineEditAltitudeApproachStart, SIGNAL(textChanged(QString)),
			this, SIGNAL(sigApproachStartingAltitudeChanged(QString)));
	connect(lineEditAGLLifeSaver, SIGNAL(textChanged(QString)),
			this, SIGNAL(sigLifeSaverHeightChanged(QString)));

	//do this  in the next turn of the event loop to have everything wired together already
	QTimer::singleShot(0, this, SLOT(readSettings()));
}

//Implementation of protected functions
void DubinsPilotDialog::closeEvent(QCloseEvent *event) {
	if (allowSave()) {
		writeSettings();
		event->accept();
	} else {
		event->accept();	//indeed, we want to close
//		event->ignore();
	}
}

void DubinsPilotDialog::reject(){
	std::cout<<"Ignoring the ESCAPE key!\nUse the 'Close' button to quit!\n";	//just ignore the escape key
}

void DubinsPilotDialog::setXPlaneConnection(bool active) {
	if (ledIndicator->isChecked() != active) {
		qDebug() << "XPlane Connection changed" << endl;
		ledIndicator->setChecked(active);
		ledIndicatorRoll->setChecked(active);
		ledIndicatorHeading->setChecked(active);
		ledIndicatorCircle->setChecked(active);
		checkBoxClimbController->setEnabled(active);
		checkBoxRollController->setEnabled(active);
		checkBoxHeadingController->setEnabled(active);
		checkBoxCircleController->setEnabled(active);
		if (!active)
			checkBoxClimbController->setChecked(false);
		if (!active)
			checkBoxRollController->setChecked(false);
		if (!active)
			checkBoxHeadingController->setChecked(false);
		if (!active)
			checkBoxCircleController->setChecked(false);
		if (loggingActive) {
			logButtonClicked();	//close the logfile;
		}
		toggleLogButton->setEnabled(active);
	}
}

void DubinsPilotDialog::attachControllerCurve(ctrlType c,
		QwtPlotCurve* ctrlCurve) {
	switch (c) {
	case ctrlType::HEADING_CONTROL:
		qDebug() << "Heading curve attached\n";
		ctrlCurve->attach(qwtPlotHeading);
		break;
	case ctrlType::RADIUS_CONTROL:
		qDebug() << "Radius curve attached\n";
		ctrlCurve->attach(qwtPlotCircle);
		break;
	case ctrlType::ROLL_CONTROL:
		qDebug() << "Roll curve attached\n";
		ctrlCurve->attach(qwtPlotRoll);
		break;
	case ctrlType::CLIMB_CONTROL:
		ctrlCurve->attach(qwtPlotClimb);
		qDebug() << "Speed curve attached\n";
		break;
	}
}

void DubinsPilotDialog::replotControllerCurve(ctrlType c) {
	switch (c) {
	case ctrlType::HEADING_CONTROL:
		qwtPlotHeading->replot();
		break;
	case ctrlType::RADIUS_CONTROL:
		qwtPlotCircle->replot();
		break;
	case ctrlType::ROLL_CONTROL:
		qwtPlotRoll->replot();
		break;
	case ctrlType::CLIMB_CONTROL:
		qwtPlotClimb->replot();
		break;
	}
}

void DubinsPilotDialog::clearInputFields(void) {
	lineEditForward->clear();
	lineEditRight->clear();
	lineEditHeight->clear();
	lineEditRotation->clear();
	elfInputFieldsDirty = false;
	this->isElfSet = false;
	pushButtonFlyPath->setEnabled(false);
	pushButtonFlyPath->setText("Take me\ndown");
	emit sigResetElf();
}

//void PIDParametersDialog::showOriginCoords(Position_WGS84 origin) {
//	pushButtonResetOrigin->setText("ResetOrigin");
//	labelLatOrigin->setText("Lat: " + QString::number(origin.lati, 'g'));
//	labelLonOrigin->setText("Lon: " + QString::number(origin.longi, 'g'));
//}

void DubinsPilotDialog::showElfCoords(Position_WGS84 elf, double elfHeading, bool pathFeasible) {
	this->elfPosition = elf;
	this->elfHeading = elfHeading;
	labelLatElf->setText("Lat: " +
			QString::number(elf.lati, 'g'));
	labelLonElf->setText("Lon: " +
			QString::number(elf.longi, 'g'));
	labelElevationElf->setText("Elevation: " +
			QString::number(elf.altitude, 'g'));
	labelHeadingElf->setText("heading: " +
			QString::number(elfHeading, 'g'));
	pushButtonFlyPath->setEnabled(pathFeasible);
	ledIndicatorPathFeasible->setChecked(pathFeasible);

#if defined(GENERATE_FLIGHT_TEST_VALUES) || defined(GENERATE_CIRCLE_TEST_VALUES)
	int lifeSaver = ((elf.altitude-150)/0.3048);
	lineEditAGLLifeSaver->setText(QString::number(
			(lifeSaver <= 300)? 300: lifeSaver));
#endif
}

void DubinsPilotDialog::setDValue(void) {
	double mantissa = doubleSpinBoxDValue->value();
	int exp = spinBoxExpDValue->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditDParameter->setText(QString::number(value, 'g'));
	valDClimb = value;
	emit sigPidParametersChanged(ctrlType::CLIMB_CONTROL, valPClimb, valIClimb,
			valDClimb);
}
void DubinsPilotDialog::setIValue(void) {
	double mantissa = doubleSpinBoxIValue->value();
	int exp = spinBoxExpIValue->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditIParameter->setText(QString::number(value, 'g'));
	valIClimb = value;
	emit sigPidParametersChanged(ctrlType::CLIMB_CONTROL, valPClimb, valIClimb,
			valDClimb);
}
void DubinsPilotDialog::setPValue(void) {
	double mantissa = doubleSpinBoxPValue->value();
	int exp = spinBoxExpPValue->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditPParameter->setText(QString::number(value, 'g'));
	valPClimb = value;
	emit sigPidParametersChanged(ctrlType::CLIMB_CONTROL, valPClimb, valIClimb,
			valDClimb);
}

void DubinsPilotDialog::setDSpinners(QString text) {
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
		valDClimb = value;
	}
}

void DubinsPilotDialog::setPSpinners(QString text) {
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
		valPClimb = value;
	}
}

void DubinsPilotDialog::setISpinners(QString text) {
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
		valIClimb = value;
	}
}

void DubinsPilotDialog::setClimbRateLabel(double climbRate) {
	valClimbRate = climbRate;
	climbRateLabel->setText(
			"gamma: " + QString::number(climbRate) + " [°]\n"
					+ QString::number(-1 / tan(deg2rad(climbRate))) + " [m/m]");
	emit sigRequestedSetValueChanged(ctrlType::CLIMB_CONTROL, valClimbRate);
	climbMarker->setYValue(valClimbRate);
	climbMarker->setLabel("sink = " + QString::number(valClimbRate));
	qwtPlotClimb->replot();
}

void DubinsPilotDialog::setClimbRateStraightLabel(double climbRate) {
	valClimbRateStraight = climbRate;
	climbRateStraightLabel->setText(
			"gamma: " + QString::number(climbRate) + " [°]\n"
					+ QString::number(-1 / tan(deg2rad(climbRate))) + " [m/m]");
	emit sigFlightPathCharacteristicsChanged(valClimbRateStraight, valClimbRateCircle, valAutoCircleRadius);
}

void DubinsPilotDialog::setClimbRateCircleLabel(double climbRate) {
	valClimbRateCircle = climbRate;
	climbRateCircleLabel->setText(
			"gamma: " + QString::number(climbRate) + " [°]\n"
					+ QString::number(-1 / tan(deg2rad(climbRate))) + " [m/m]");
	emit sigFlightPathCharacteristicsChanged(valClimbRateStraight, valClimbRateCircle, valAutoCircleRadius);
}

void DubinsPilotDialog::setAutoCircleRadiusLabel(double radius) {
	valAutoCircleRadius = radius;
	autoCircleRadiusLabel->setText("radius = " + QString::number(valAutoCircleRadius));
	emit sigFlightPathCharacteristicsChanged(valClimbRateStraight, valClimbRateCircle, valAutoCircleRadius);
}

void DubinsPilotDialog::setTargetValueControlKnob(ctrlType ctrl, double targetValue, bool isLeftCircle){
	switch (ctrl) {
	case ctrlType::CLIMB_CONTROL:
		KnobClimb->setValue(targetValue);
		break;
	case ctrlType::ROLL_CONTROL:
		KnobRoll->setValue(targetValue);
		break;
	case ctrlType::RADIUS_CONTROL:
		KnobCircle->setValue(targetValue);
		isLeftCircle ?
				radioButtonCircleLeft->setChecked(true) :
				radioButtonCircleRight->setChecked(true);
		break;
	case ctrlType::HEADING_CONTROL:
		CompassHeading->setValue(targetValue);
		break;
	}
}

void DubinsPilotDialog::setControllerCheckButtons(ctrlType ctrl, bool enable,
		bool checked) {
	switch (ctrl) {
	case ctrlType::CLIMB_CONTROL:
		checkBoxClimbController->setEnabled(enable);
		checkBoxClimbController->setChecked(checked);
		break;
	case ctrlType::ROLL_CONTROL:
		checkBoxRollController->setEnabled(enable);
		checkBoxRollController->setChecked(checked);
		break;
	case ctrlType::RADIUS_CONTROL:
		checkBoxCircleController->setEnabled(enable);
		checkBoxCircleController->setChecked(checked);
		break;
	case ctrlType::HEADING_CONTROL:
		checkBoxHeadingController->setEnabled(enable);
		checkBoxHeadingController->setChecked(checked);
		break;
	}
}


void DubinsPilotDialog::setDValueBank(void) {
	double mantissa = doubleSpinBoxDValueBank->value();
	int exp = spinBoxExpDValueBank->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditDParameterBank->setText(QString::number(value, 'g'));
	valDRoll = value;
	emit sigPidParametersChanged(ctrlType::ROLL_CONTROL, valPRoll, valIRoll,
			valDRoll);
}
void DubinsPilotDialog::setIValueBank(void) {
	double mantissa = doubleSpinBoxIValueBank->value();
	int exp = spinBoxExpIValueBank->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditIParameterBank->setText(QString::number(value, 'g'));
	valIRoll = value;
	emit sigPidParametersChanged(ctrlType::ROLL_CONTROL, valPRoll, valIRoll,
			valDRoll);
}
void DubinsPilotDialog::setPValueBank(void) {
	double mantissa = doubleSpinBoxPValueBank->value();
	int exp = spinBoxExpPValueBank->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditPParameterBank->setText(QString::number(value, 'g'));
	valPRoll = value;
	emit sigPidParametersChanged(ctrlType::ROLL_CONTROL, valPRoll, valIRoll,
			valDRoll);
}

void DubinsPilotDialog::setDSpinnersBank(QString text) {
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
		valDRoll = value;
	}
}

void DubinsPilotDialog::setPSpinnersBank(QString text) {
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
		valPRoll = value;
	}
}

void DubinsPilotDialog::setISpinnersBank(QString text) {
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
		valIRoll = value;
	}
}

void DubinsPilotDialog::setBankingLabel(double roll) {
	valRoll = roll;
	rollLabel->setText("Roll: " + QString::number(valRoll) + " [deg]");
	emit sigRequestedSetValueChanged(ctrlType::ROLL_CONTROL, roll);
	rollMarker->setYValue(valRoll);
	rollMarker->setLabel("roll = " + QString::number(valRoll));
	qwtPlotRoll->replot();
}

void DubinsPilotDialog::setDValueHeading(void) {
	double mantissa = doubleSpinBoxDValueHeading->value();
	int exp = spinBoxExpDValueHeading->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditDParameterHeading->setText(QString::number(value, 'g'));
	valDHeading = value;
	emit sigPidParametersChanged(ctrlType::HEADING_CONTROL, valPHeading,
			valIHeading, valDHeading);
}
void DubinsPilotDialog::setIValueHeading(void) {
	double mantissa = doubleSpinBoxIValueHeading->value();
	int exp = spinBoxExpIValueHeading->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditIParameterHeading->setText(QString::number(value, 'g'));
	valIHeading = value;
	emit sigPidParametersChanged(ctrlType::HEADING_CONTROL, valPHeading,
			valIHeading, valDHeading);
}
void DubinsPilotDialog::setPValueHeading(void) {
	double mantissa = doubleSpinBoxPValueHeading->value();
	int exp = spinBoxExpPValueHeading->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditPParameterHeading->setText(QString::number(value, 'g'));
	valPHeading = value;
	emit sigPidParametersChanged(ctrlType::HEADING_CONTROL, valPHeading,
			valIHeading, valDHeading);
}

void DubinsPilotDialog::setDSpinnersHeading(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxDValueHeading->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxDValueHeading->setValue(mantissa);
		spinBoxExpDValueHeading->setValue(exp);
		valDHeading = value;
	}
}

void DubinsPilotDialog::setPSpinnersHeading(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxPValueHeading->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxPValueHeading->setValue(mantissa);
		spinBoxExpPValueHeading->setValue(exp);
		valPHeading = value;
	}
}

void DubinsPilotDialog::setISpinnersHeading(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxIValueHeading->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxIValueHeading->setValue(mantissa);
		spinBoxExpIValueHeading->setValue(exp);
		valIHeading = value;
	}
}

void DubinsPilotDialog::setHeadingLabel(double heading) {
	valHeading = heading;
	headingLabel->setText("Heading: " + QString::number(valHeading) + " [deg]");
	emit sigRequestedSetValueChanged(ctrlType::HEADING_CONTROL, heading);
	headingMarker->setYValue(valHeading);
	headingMarker->setLabel("heading = " + QString::number(valHeading));
	qwtPlotHeading->replot();
}

void DubinsPilotDialog::setDValueCircle(void) {
	double mantissa = doubleSpinBoxDValueCircle->value();
	int exp = spinBoxExpDValueCircle->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditDParameterCircle->setText(QString::number(value, 'g'));
	valDCircle = value;
	emit sigPidParametersChanged(ctrlType::RADIUS_CONTROL, valPCircle,
			valICircle, valDCircle);
}
void DubinsPilotDialog::setIValueCircle(void) {
	double mantissa = doubleSpinBoxIValueCircle->value();
	int exp = spinBoxExpIValueCircle->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditIParameterCircle->setText(QString::number(value, 'g'));
	valICircle = value;
	emit sigPidParametersChanged(ctrlType::RADIUS_CONTROL, valPCircle,
			valICircle, valDCircle);
}
void DubinsPilotDialog::setPValueCircle(void) {
	double mantissa = doubleSpinBoxPValueCircle->value();
	int exp = spinBoxExpPValueCircle->value();
	double value = mantissa * exp10(exp);
//	qDebug()<< "value changed: " << value << endl;
	lineEditPParameterCircle->setText(QString::number(value, 'g'));
	valPCircle = value;
	emit sigPidParametersChanged(ctrlType::RADIUS_CONTROL, valPCircle,
			valICircle, valDCircle);
}

void DubinsPilotDialog::setDSpinnersCircle(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxDValueCircle->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxDValueCircle->setValue(mantissa);
		spinBoxExpDValueCircle->setValue(exp);
		valDCircle = value;
	}
}

void DubinsPilotDialog::setPSpinnersCircle(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxPValueCircle->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxPValueCircle->setValue(mantissa);
		spinBoxExpPValueCircle->setValue(exp);
		valPCircle = value;
	}
}

void DubinsPilotDialog::setISpinnersCircle(QString text) {
	double value = text.toDouble();
	double exp = floor(log10(fabs(value)));
	if (exp < minExp - 1) {
		doubleSpinBoxIValueCircle->setValue(0.0);
		return;
	}
	double mantissa = value / exp10(exp);
	if (minExp <= exp && exp <= maxExp) {
		doubleSpinBoxIValueCircle->setValue(mantissa);
		spinBoxExpIValueCircle->setValue(exp);
		valICircle = value;
	}
}

void DubinsPilotDialog::setCircleLabel(double radius) {
	valCircle = radius;
	circleRadiusLabel->setText(
			"radius: " + QString::number(valCircle) + " [m]");
	emit sigRequestedSetValueChanged(ctrlType::RADIUS_CONTROL, valCircle);
	circleMarker->setYValue(valCircle);
	circleMarker->setLabel("radius = " + QString::number(valCircle));
	qwtPlotCircle->replot();
}

void DubinsPilotDialog::tempomatActiveStateChanged(bool active) {
	emit sigPidParametersChanged(ctrlType::CLIMB_CONTROL, valPClimb, valIClimb,
			valDClimb);
	emit sigRequestedSetValueChanged(ctrlType::CLIMB_CONTROL, valClimbRate);
	emit sigCtrlActiveStateChanged(ctrlType::CLIMB_CONTROL, active);
}

void DubinsPilotDialog::rollControlActiveStateChanged(bool active) {
	emit sigRequestedSetValueChanged(ctrlType::ROLL_CONTROL, valRoll);
	emit sigPidParametersChanged(ctrlType::ROLL_CONTROL, valPRoll, valIRoll,
			valDRoll);
	emit sigCtrlActiveStateChanged(ctrlType::ROLL_CONTROL, active);
}

void DubinsPilotDialog::headingControlActiveStateChanged(bool active) {
	emit sigRequestedSetValueChanged(ctrlType::HEADING_CONTROL, valHeading);
	emit sigPidParametersChanged(ctrlType::HEADING_CONTROL, valPHeading,
			valIHeading, valDHeading);
	emit sigCtrlActiveStateChanged(ctrlType::HEADING_CONTROL, active);

	DataCenter* dc = DataCenter::getInstance();
	dc->changeSegmentStatisticsState(false);	// to be sure to start new stats
	if(active){
		dc->changeSegmentStatisticsState(true);
	}
}
void DubinsPilotDialog::circleControlActiveStateChanged(bool active) {
	emit sigRequestedSetValueChanged(ctrlType::RADIUS_CONTROL, valCircle);
	emit sigPidParametersChanged(ctrlType::RADIUS_CONTROL, valPCircle,
			valICircle, valDCircle);
	emit sigCircleDirectionChanged(radioButtonCircleLeft->isChecked(),
			valCircle);
	emit sigCtrlActiveStateChanged(ctrlType::RADIUS_CONTROL, active);

	DataCenter* dc = DataCenter::getInstance();
	dc->changeSegmentStatisticsState(false);	// to be sure to start new stats
	if(active){
		dc->changeSegmentStatisticsState(true);
	}
}

void DubinsPilotDialog::radioButtonCircleClicked(void) {
	emit sigCircleDirectionChanged(radioButtonCircleLeft->isChecked(),
			valCircle);
}

void DubinsPilotDialog::continuousDubinsCalc(bool active){
	if(active){
		contCalcTimer->start(timerMilliseconds);
	} else {
		contCalcTimer->stop();
	}
}

void DubinsPilotDialog::contCalcTimerExpired(void) {
	submitElfData();
}


void DubinsPilotDialog::readSettings() {
	QSettings settings("EEE", "PID_Parameters");
	bool tempActive=false;

	qDebug() << "reading settings from " << settings.fileName() << endl;

	restoreGeometry(settings.value("geometry").toByteArray());

	valPClimb = settings.value("valPClimb").toDouble();
	qDebug() << "valP: " << valPClimb;
	lineEditPParameter->setText(QString::number(valPClimb, 'g'));
	valIClimb = settings.value("valIClimb").toDouble();
	qDebug() << "\tvalI: " << valIClimb;
	lineEditIParameter->setText(QString::number(valIClimb, 'g'));
	valDClimb = settings.value("valDClimb").toDouble();
	qDebug() << "\tvalD: " << valDClimb;
	lineEditDParameter->setText(QString::number(valDClimb, 'g'));
	valClimbRate = settings.value("valClimbRate").toDouble();
	KnobClimb->setValue(valClimbRate);
	setClimbRateLabel(valClimbRate);//call it manually, in case read in value is 0. then nothing changes and no marker update is performed.
	tempActive = settings.value("climbControlActive", false).toBool();
	checkBoxClimbController->setChecked(tempActive);
	tempomatActiveStateChanged(tempActive);
	qDebug() << "\tvalClimbRate: " << valClimbRate << endl;
//	checkBoxClimbController->setChecked(
//			settings.value("climbControlActive").toBool());

	valPRoll = settings.value("valPRoll").toDouble();
	qDebug() << "valPBank: " << valPRoll;
	lineEditPParameterBank->setText(QString::number(valPRoll, 'g'));
	valIRoll = settings.value("valIRoll").toDouble();
	qDebug() << "\tvalIBank: " << valIRoll;
	lineEditIParameterBank->setText(QString::number(valIRoll, 'g'));
	valDRoll = settings.value("valDRoll").toDouble();
	qDebug() << "\tvalDBank: " << valDRoll;
	lineEditDParameterBank->setText(QString::number(valDRoll, 'g'));
	valRoll = settings.value("valRoll").toDouble();
	KnobRoll->setValue(valRoll);
	setBankingLabel(valRoll);//call it manually, in case read in value is 0. then nothing changes and no marker update is performed.
	tempActive = settings.value("rollControlActive", false).toBool();
	checkBoxRollController->setChecked(tempActive);
	rollControlActiveStateChanged(tempActive);
	qDebug() << "\tvalBanking: " << valRoll << endl;
//	checkBoxRollController->setChecked(
//			settings.value("rollControlActive").toBool());

	valPHeading = settings.value("valPHeading").toDouble();
	lineEditPParameterHeading->setText(QString::number(valPHeading, 'g'));
	valIHeading = settings.value("valIHeading").toDouble();
	lineEditIParameterHeading->setText(QString::number(valIHeading, 'g'));
	valDHeading = settings.value("valDHeading").toDouble();
	lineEditDParameterHeading->setText(QString::number(valDHeading, 'g'));
	valHeading = settings.value("valHeading").toDouble();
	CompassHeading->setValue(valHeading);
	setHeadingLabel(valHeading);//call it manually, in case read in value is 0. then nothing changes and no marker update is performed.
	tempActive = settings.value("headingControlActive", false).toBool();
	checkBoxHeadingController->setChecked(tempActive);
	headingControlActiveStateChanged(tempActive);

	valPCircle = settings.value("valPCircle").toDouble();
	lineEditPParameterCircle->setText(QString::number(valPCircle, 'g'));
	valICircle = settings.value("valICircle").toDouble();
	lineEditIParameterCircle->setText(QString::number(valICircle, 'g'));
	valDCircle = settings.value("valDCircle").toDouble();
	lineEditDParameterCircle->setText(QString::number(valDCircle, 'g'));
	valCircle = settings.value("valCircle").toDouble();
	KnobCircle->setValue(valCircle);
	setCircleLabel(valCircle);//call it manually, in case read in value is 0. then nothing changes and no marker update is performed.
	tempActive = settings.value("circleControlActive", false).toBool();
	checkBoxCircleController->setChecked(tempActive);
	circleControlActiveStateChanged(tempActive);

	valClimbRateStraight = settings.value("valClimbRateStraight").toDouble();
	KnobClimbStraight->setValue(valClimbRateStraight);
	setClimbRateStraightLabel(valClimbRateStraight);

	valClimbRateCircle = settings.value("valClimbRateCircle").toDouble();
	KnobClimbCircle->setValue(valClimbRateCircle);
	setClimbRateCircleLabel(valClimbRateCircle);

	valAutoCircleRadius = settings.value("valAutoCircleRadius").toDouble();
	KnobAutoCircleRadius->setValue(valAutoCircleRadius);
	setAutoCircleRadiusLabel(valAutoCircleRadius);

	initialLogFileDir = settings.value("initialLogFileDir").toString();
	if (initialLogFileDir.isEmpty())
		initialLogFileDir = QDir::homePath();
	lineEditFileName->setText(initialLogFileDir);
}

void DubinsPilotDialog::writeSettings() {
	QSettings settings("EEE", "PID_Parameters");
	qDebug() << "Saving: valP: " << valPClimb;
	qDebug() << "\tvalI: " << valIClimb;
	qDebug() << "\tvalD: " << valDClimb;
	qDebug() << "\tvalClimbRate: " << valClimbRate << endl;

	settings.setValue("valPClimb", QVariant::fromValue(valPClimb));
	settings.setValue("valIClimb", QVariant::fromValue(valIClimb));
	settings.setValue("valDClimb", QVariant::fromValue(valDClimb));
	settings.setValue("valClimbRate", QVariant::fromValue(valClimbRate));
	settings.setValue("climbControlActive",
			QVariant::fromValue((checkBoxClimbController->isChecked())));

	settings.setValue("valPRoll", QVariant::fromValue(valPRoll));
	settings.setValue("valIRoll", QVariant::fromValue(valIRoll));
	settings.setValue("valDRoll", QVariant::fromValue(valDRoll));
	settings.setValue("valRoll", QVariant::fromValue(valRoll));
	settings.setValue("rollControlActive",
			QVariant::fromValue((checkBoxRollController->isChecked())));


	settings.setValue("valPHeading", QVariant::fromValue(valPHeading));
	settings.setValue("valIHeading", QVariant::fromValue(valIHeading));
	settings.setValue("valDHeading", QVariant::fromValue(valDHeading));
	settings.setValue("valHeading", QVariant::fromValue(valHeading));

	settings.setValue("valPCircle", QVariant::fromValue(valPCircle));
	settings.setValue("valICircle", QVariant::fromValue(valICircle));
	settings.setValue("valDCircle", QVariant::fromValue(valDCircle));
	settings.setValue("valCircle", QVariant::fromValue(valCircle));

	settings.setValue("valClimbRateCircle", QVariant::fromValue(valClimbRateCircle));
	settings.setValue("valClimbRateStraight", QVariant::fromValue(valClimbRateStraight));
	settings.setValue("valAutoCircleRadius", QVariant::fromValue(valAutoCircleRadius));


	settings.setValue("initialLogFileDir",
			QVariant::fromValue(initialLogFileDir));
}

void DubinsPilotDialog::setAltitude(void) {
	double altitudeAboveGround = lineEditAltitude->text().toDouble();
	emit sigSendXPDataRef("sim/flightmodel/position/local_y",
			altitudeAboveGround);
}
void DubinsPilotDialog::setIgniterOff(void) {
	//see X-Plane failure enum here: http://www.xsquawkbox.net/xpsdk/mediawiki/Failure_Modeling
	const int INOPERATIVE_NOW = 6;
	emit sigSendXPDataRef("sim/operation/failures/rel_engfai0",
			INOPERATIVE_NOW);
}

void DubinsPilotDialog::logButtonClicked() {
	if (loggingActive) {
		emit sigLoggingActiveStateChanged(false, QDir(""), "");
		setLoggingState(false);
	} else {
		//start logging
		emit sigLoggingActiveStateChanged(true, initialLogFileDir, generateLogfilename());
		setLoggingState(true);
	}
}

void DubinsPilotDialog::setLoggingState(bool _loggingActive){
	loggingActive = _loggingActive;
	if(!loggingActive){
		//stop logging
		lineEditFileName->setEnabled(true);
		choseFileNameButton->setEnabled(true);
		toggleLogButton->setText(tr("Start &Logging"));
	} else {
		//start logging
		choseFileNameButton->setEnabled(false);
		lineEditFileName->setEnabled(false);
		toggleLogButton->setText(tr("Stop &Logging"));
	}
}

void DubinsPilotDialog::fileNameSelectorClicked(void) {
	initialLogFileDir = QFileDialog::getExistingDirectory(this,
			tr("Open Directory for logging:"), initialLogFileDir,
			QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	logFile = generateLogfilename();
	lineEditFileName->setText(
			QFileInfo(initialLogFileDir, logFile).absolutePath());
}

void DubinsPilotDialog::fileNameEditingFinished() {
	QFileInfo cleanPath = QFileInfo(lineEditFileName->text());
	qDebug() << "Set logfile to: " << cleanPath.absoluteFilePath() << endl;
	initialLogFileDir = cleanPath.filePath();
	logFile = cleanPath.fileName();
	lineEditFileName->setText(cleanPath.absoluteFilePath());
}

void DubinsPilotDialog::submitElfData(void) {
	double forward, right, height, rotation;
	forward = lineEditForward->text().toDouble();
	right = lineEditRight->text().toDouble();
	height = lineEditHeight->text().toDouble();
	rotation = fmod(lineEditRotation->text().toDouble(), 360.0);

	if (radioButtonRelative->isChecked()
			&& (elfInputFieldsDirty || !isElfSet
					|| checkBoxAlwaysRelative->isChecked())) {
		emit sigSetElfLocation(forward, right, height, rotation,
				radioButtonLSL->isChecked()?pathTypeEnum::LSL:pathTypeEnum::RSR);
		isElfSet = true;
		elfInputFieldsDirty = false;
	} else if (radioButtonAbsolute->isChecked()){
		Position_WGS84 elf=Position_WGS84(forward, right, height);
		emit sigSetElfLocation(elf, rotation,
				radioButtonLSL->isChecked()?pathTypeEnum::LSL:pathTypeEnum::RSR);
	} else {
		emit sigSetElfLocation(elfPosition, elfHeading,
				radioButtonLSL->isChecked()?pathTypeEnum::LSL:pathTypeEnum::RSR);
	}
}

void DubinsPilotDialog::toggleAbsoluteRelative(void){
	clearInputFields();
	if(radioButtonRelative->isChecked()){
		labelForward->setText("forward:");
		labelUnitForward->setText("m");
		labelRight->setText("right:");
		labelUnitRight->setText("m");
		labelHeight->setText("height:");
		labelRotation->setText("rotation:");
		checkBoxAlwaysRelative->setEnabled(true);
	} else {
		labelForward->setText("lati:");
		labelUnitForward->setText("°N");
		labelRight->setText("longi:");
		labelUnitRight->setText("°E");
		labelHeight->setText("elevation:");
		labelRotation->setText("heading:");
		checkBoxAlwaysRelative->setEnabled(false);
	}
}


void DubinsPilotDialog::takeMeDown(void){
	if(!isPathTracking){
		checkBoxCont->setChecked(false);	// we need to stop the continuous calculation
		checkBoxCont->setEnabled(false);	// we need to stop the continuous calculation
		submitElfData();	// we recalculate the path directly before tracking it
		emit sigStartPathTracking(true);
		isPathTracking = true;
		//TODO Da müsste man eigentlich noch die ganzenEingabefelder disablen bis angekommen oder Cancel
	} else {
		emit sigStartPathTracking(false);
		DataCenter::getInstance()->changeSegmentStatisticsState(false);	//nicht die schönste Stelle, aber sollte gehen
		isPathTracking = false;
	}
	displayPathTrackingStatus(isPathTracking);
}

void DubinsPilotDialog::displayPathTrackingStatus(bool _isPathTracking){
	isPathTracking = _isPathTracking;	//this should be in sync with above, but to be sure
	isPathTracking ?
			pushButtonFlyPath->setText("Cancel"):
			pushButtonFlyPath->setText("Take me\ndown");
	if(isPathTracking){
		checkBoxCont->setChecked(false);	// we need to stop the continuous calculation
		checkBoxCont->setEnabled(false);
		contCalcTimer->stop();
	} else {
		checkBoxCont->setEnabled(true);
	}
}

void DubinsPilotDialog::clickSetHeight(void){	//to enable the "Life Saver" when Altitude AGL is too low
	if(loggingActive){
		toggleLogButton->click();	//stop logging, if active
	}
	setHeightButton->click();

#ifdef GENERATE_FLIGHT_TEST_VALUES

	int forward, right, rotation, heightLoss;
	pathTypeEnum type=pathTypeEnum::LSL;
	forward = -3000 + (rand() % static_cast<int>(3000 + 3000 + 1));
	right = -3000 + (rand() % static_cast<int>(3000 + 3000 + 1));
	rotation = 0 + (rand() % static_cast<int>(360 - 0 + 1));
	type = (rand() % static_cast<int>(2)) == 0?pathTypeEnum::LSL : pathTypeEnum::RSR;

	heightLoss = (rand() % static_cast<int>(2)) == 0?
			0 : -(700 + (rand() % static_cast<int>(1300 - 700 + 1)));

	lineEditForward->setText(QString::number(forward));
	lineEditRight->setText(QString::number(right));
	lineEditHeight->setText(QString::number(heightLoss));
	lineEditRotation->setText(QString::number(rotation));
	if(type == +pathTypeEnum::LSL){
		radioButtonLSL->setChecked(true);
		radioButtonRSR->setChecked(false);
	}else {
		radioButtonLSL->setChecked(false);
		radioButtonRSR->setChecked(true);
	}
	checkBoxCont->setChecked(true);
	continuousDubinsCalc(true);
#endif

#ifdef GENERATE_CIRCLE_TEST_VALUES
	static double angle = -180;

	double twoR = 901, dist = 2000;
	double forward, right;


	int rotation = 180, heightLoss = 0;
	rotatePoint(dist, 0, angle, forward, right);
	right += twoR;
	angle += 10;

	lineEditForward->setText(QString::number(int(forward)));
	lineEditRight->setText(QString::number(int(right)));
	lineEditHeight->setText(QString::number(heightLoss));
	lineEditRotation->setText(QString::number(rotation));
	checkBoxCont->setChecked(true);
	continuousDubinsCalc(true);
#endif

}

void DubinsPilotDialog::clickTakeMeDown(void){ //to start path tracking at a defined altitude
	if(!loggingActive){
		toggleLogButton->click();	//start logging, if not already active
	}
//TODO das muss wieder raus, wenn wirklich geflogen werden soll. aktuell nur logging
		pushButtonFlyPath->click();
}

void DubinsPilotDialog::clickPause(bool isPaused){ //to pause the simulator on certain events
	if(checkBoxStopTheWorld->isChecked()){
		checkBoxPause->setChecked(isPaused);
	}
}



void DubinsPilotDialog::setWind(void) {
	// Eingabe sind Knoten, Rückgabe sind Meter pro Sekunde
	double windDirFrom = CompassWind->value();

	// TODO wieder rausnehmen
	if(checkBoxPlaneHeading->isChecked()){
		DataCenter::getInstance()->setTrue_psi(windDirFrom);
	}

	labelSelectedWindFrom->setText(
			"from "+QString::number(windDirFrom, 'g') + " [deg]");
	double windVelocityKnots = SliderWindVelocity->value();
	windVelocityKnots = windVelocityKnots+0.0;
	labelSelectedWindV->setText(
			QString::number(knots_to_ms(windVelocityKnots), 'g') + " [m/s]");
	emit sigSendXPDataRef("sim/weather/wind_altitude_msl_m[0]", 1000.0);
	emit sigSendXPDataRef("sim/weather/wind_direction_degt[0]", windDirFrom);
	emit sigSendXPDataRef("sim/weather/wind_speed_kt[0]",
			(windVelocityKnots));
	emit sigSendXPDataRef("sim/weather/wind_altitude_msl_m[1]", 2000.0);
	emit sigSendXPDataRef("sim/weather/wind_direction_degt[1]", windDirFrom);
	emit sigSendXPDataRef("sim/weather/wind_speed_kt[1]",
			(windVelocityKnots));
	emit sigSendXPDataRef("sim/weather/wind_altitude_msl_m[2]", 3000.0);
	emit sigSendXPDataRef("sim/weather/wind_direction_degt[2]", windDirFrom);
	emit sigSendXPDataRef("sim/weather/wind_speed_kt[2]",
			(windVelocityKnots));
}

void DubinsPilotDialog::displayCurrentWind(double windDirFrom, double windVelocity){
	labelCurrentWindFrom->setText(
			"from "+QString::number(windDirFrom, 'f', 1) + " [deg]");
	labelCurrentWindV->setText(
			QString::number(windVelocity, 'f', 2) + " [m/s]");
}

void DubinsPilotDialog::displayFlightPhase(QString flightPhase, QString toGo){
	labelFlightPhase->setText(flightPhase);
	labelToGo->setText(toGo);
}


QString DubinsPilotDialog::generateLogfilename() {
	QString DateString =
			QDateTime::currentDateTime().toString(Qt::ISODate).replace('-', '_').replace(
					':', '_');
	return "Log_" + DateString + ".csv";
}

void DubinsPilotDialog::pauseSimulation(bool isPaused){
//	DataCenter* dc =DataCenter::getInstance();
	emit sigSendXPDataRef("sim/time/sim_speed", isPaused?0.0:1.0);
//			dc->SendXPDataRef("sim/time/sim_speed", pause?0.0:1.0);
	emit sigSetSimulationPaused(isPaused);
}


void DubinsPilotDialog::setupPlot(void) {
	qwtPlotClimb->setTitle("Flight Path Angle / Sink Control");

// axes
	qwtPlotClimb->setAxisTitle(qwtPlotClimb->xBottom, "t [ms] -->");
	qwtPlotClimb->setAxisScale(qwtPlotClimb->xBottom, -plotDataSize, 0.0);

	qwtPlotClimb->setAxisTitle(qwtPlotClimb->yLeft,
			"Flight Path Angle [°] -->");
	qwtPlotClimb->setAxisScale(qwtPlotClimb->yLeft, -6.5, -4.5);

	qwtPlotClimb->setAxisTitle(qwtPlotClimb->yRight, "<-- elevator");
	qwtPlotClimb->setAxisScale(qwtPlotClimb->yRight, -0.75, 0.75);

	qwtPlotClimb->enableAxis(QwtPlot::xBottom);
	qwtPlotClimb->enableAxis(QwtPlot::yLeft);
	qwtPlotClimb->enableAxis(QwtPlot::yRight);

//  ...a horizontal line at y = 0...
	climbMarker = new QwtPlotMarker();
	climbMarker->setLabelAlignment(Qt::AlignRight | Qt::AlignTop);
	climbMarker->setLineStyle(QwtPlotMarker::HLine);
	climbMarker->attach(qwtPlotClimb);

	zoom_yLeftClimb = new QwtPlotMagnifier(qwtPlotClimb->canvas());
	zoom_yLeftClimb->setWheelButtonState(Qt::NoModifier);
	zoom_yLeftClimb->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yLeftClimb->setAxisEnabled(QwtPlot::yLeft, true);
	zoom_yLeftClimb->setAxisEnabled(QwtPlot::yRight, false);

	drag_yLeftClimb = new QwtPlotPanner(qwtPlotClimb->canvas());
	drag_yLeftClimb->setMouseButton(Qt::LeftButton, Qt::NoModifier);
	drag_yLeftClimb->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yLeftClimb->setAxisEnabled(QwtPlot::yLeft, true);
	drag_yLeftClimb->setAxisEnabled(QwtPlot::yRight, false);

	zoom_yRightClimb = new QwtPlotMagnifier(qwtPlotClimb->canvas());
	zoom_yRightClimb->setWheelButtonState(Qt::ControlModifier);
	zoom_yRightClimb->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yRightClimb->setAxisEnabled(QwtPlot::yLeft, false);
	zoom_yRightClimb->setAxisEnabled(QwtPlot::yRight, true);

	drag_yRightClimb = new QwtPlotPanner(qwtPlotClimb->canvas());
	drag_yRightClimb->setMouseButton(Qt::LeftButton, Qt::ControlModifier);
	drag_yRightClimb->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yRightClimb->setAxisEnabled(QwtPlot::yLeft, false);
	drag_yRightClimb->setAxisEnabled(QwtPlot::yRight, true);

	qwtPlotClimb->replot();
	qwtPlotClimb->show();

	qwtPlotRoll->setTitle("Roll Angle");

// axes
	qwtPlotRoll->setAxisTitle(qwtPlotRoll->xBottom, "t [ms] -->");
	qwtPlotRoll->setAxisScale(qwtPlotRoll->xBottom, -plotDataSize, 0.0);

	qwtPlotRoll->setAxisTitle(qwtPlotRoll->yLeft, "Roll angle [deg] -->");
	qwtPlotRoll->setAxisScale(qwtPlotRoll->yLeft, -30, 30);

	qwtPlotRoll->setAxisTitle(qwtPlotRoll->yRight, "<-- aileron");
	qwtPlotRoll->setAxisScale(qwtPlotRoll->yRight, -0.75, 0.75);

	qwtPlotRoll->enableAxis(QwtPlot::xBottom);
	qwtPlotRoll->enableAxis(QwtPlot::yLeft);
	qwtPlotRoll->enableAxis(QwtPlot::yRight);

//  ...a horizontal line at y = 0...
	rollMarker = new QwtPlotMarker();
	rollMarker->setLabelAlignment(Qt::AlignRight | Qt::AlignTop);
	rollMarker->setLineStyle(QwtPlotMarker::HLine);
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

	qwtPlotCircle->setTitle("Circle Radius");

// axes
	qwtPlotCircle->setAxisTitle(qwtPlotCircle->xBottom, "t [ms] -->");
	qwtPlotCircle->setAxisScale(qwtPlotCircle->xBottom, -plotDataSize, 0.0);

	qwtPlotCircle->setAxisTitle(qwtPlotCircle->yLeft, "Circle Radius [m] -->");
	qwtPlotCircle->setAxisScale(qwtPlotCircle->yLeft, 380, 520);

	qwtPlotCircle->setAxisTitle(qwtPlotCircle->yRight,
			"<-- Roll Correction [deg]");
	qwtPlotCircle->setAxisScale(qwtPlotCircle->yRight, -5, 5);

	qwtPlotCircle->enableAxis(QwtPlot::xBottom);
	qwtPlotCircle->enableAxis(QwtPlot::yLeft);
	qwtPlotCircle->enableAxis(QwtPlot::yRight);

//  ...a horizontal line at y = 0...
	circleMarker = new QwtPlotMarker();
	circleMarker->setLabelAlignment(Qt::AlignRight | Qt::AlignTop);
	circleMarker->setLineStyle(QwtPlotMarker::HLine);
	circleMarker->attach(qwtPlotCircle);

	zoom_yLeftCircle = new QwtPlotMagnifier(qwtPlotCircle->canvas());
	zoom_yLeftCircle->setWheelButtonState(Qt::NoModifier);
	zoom_yLeftCircle->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yLeftCircle->setAxisEnabled(QwtPlot::yLeft, true);
	zoom_yLeftCircle->setAxisEnabled(QwtPlot::yRight, false);

	drag_yLeftCircle = new QwtPlotPanner(qwtPlotCircle->canvas());
	drag_yLeftCircle->setMouseButton(Qt::LeftButton, Qt::NoModifier);
	drag_yLeftCircle->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yLeftCircle->setAxisEnabled(QwtPlot::yLeft, true);
	drag_yLeftCircle->setAxisEnabled(QwtPlot::yRight, false);

	zoom_yRightCircle = new QwtPlotMagnifier(qwtPlotCircle->canvas());
	zoom_yRightCircle->setWheelButtonState(Qt::ControlModifier);
	zoom_yRightCircle->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yRightCircle->setAxisEnabled(QwtPlot::yLeft, false);
	zoom_yRightCircle->setAxisEnabled(QwtPlot::yRight, true);

	drag_yRightCircle = new QwtPlotPanner(qwtPlotCircle->canvas());
	drag_yRightCircle->setMouseButton(Qt::LeftButton, Qt::ControlModifier);
	drag_yRightCircle->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yRightCircle->setAxisEnabled(QwtPlot::yLeft, false);
	drag_yRightCircle->setAxisEnabled(QwtPlot::yRight, true);

	qwtPlotCircle->replot();
	qwtPlotCircle->show();

	qwtPlotHeading->setTitle("Heading");

// axes
	qwtPlotHeading->setAxisTitle(qwtPlotHeading->xBottom, "t [ms] -->");
	qwtPlotHeading->setAxisScale(qwtPlotHeading->xBottom, -plotDataSize, 0.0);

	qwtPlotHeading->setAxisTitle(qwtPlotHeading->yLeft,
			"Heading Angle [deg] -->");
	qwtPlotHeading->setAxisScale(qwtPlotHeading->yLeft, 0, 360);

	qwtPlotHeading->setAxisTitle(qwtPlotHeading->yRight,
			"<-- Roll Setpoint [deg]");
	qwtPlotHeading->setAxisScale(qwtPlotHeading->yRight, -30, 30);

	qwtPlotHeading->enableAxis(QwtPlot::xBottom);
	qwtPlotHeading->enableAxis(QwtPlot::yLeft);
	qwtPlotHeading->enableAxis(QwtPlot::yRight);

//  ...a horizontal line at y = 0...
	headingMarker = new QwtPlotMarker();
	headingMarker->setLabelAlignment(Qt::AlignRight | Qt::AlignTop);
	headingMarker->setLineStyle(QwtPlotMarker::HLine);
	headingMarker->attach(qwtPlotHeading);

	zoom_yLeftHeading = new QwtPlotMagnifier(qwtPlotHeading->canvas());
	zoom_yLeftHeading->setWheelButtonState(Qt::NoModifier);
	zoom_yLeftHeading->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yLeftHeading->setAxisEnabled(QwtPlot::yLeft, true);
	zoom_yLeftHeading->setAxisEnabled(QwtPlot::yRight, false);

	drag_yLeftHeading = new QwtPlotPanner(qwtPlotHeading->canvas());
	drag_yLeftHeading->setMouseButton(Qt::LeftButton, Qt::NoModifier);
	drag_yLeftHeading->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yLeftHeading->setAxisEnabled(QwtPlot::yLeft, true);
	drag_yLeftHeading->setAxisEnabled(QwtPlot::yRight, false);

	zoom_yRightHeading = new QwtPlotMagnifier(qwtPlotHeading->canvas());
	zoom_yRightHeading->setWheelButtonState(Qt::ControlModifier);
	zoom_yRightHeading->setAxisEnabled(QwtPlot::xBottom, false);
	zoom_yRightHeading->setAxisEnabled(QwtPlot::yLeft, false);
	zoom_yRightHeading->setAxisEnabled(QwtPlot::yRight, true);

	drag_yRightHeading = new QwtPlotPanner(qwtPlotHeading->canvas());
	drag_yRightHeading->setMouseButton(Qt::LeftButton, Qt::ControlModifier);
	drag_yRightHeading->setAxisEnabled(QwtPlot::xBottom, false);
	drag_yRightHeading->setAxisEnabled(QwtPlot::yLeft, false);
	drag_yRightHeading->setAxisEnabled(QwtPlot::yRight, true);

	qwtPlotHeading->replot();
	qwtPlotHeading->show();
}
