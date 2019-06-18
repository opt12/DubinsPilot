/*
 * Gym.h
 *
 *  Created on: 03.06.2019
 *      Author: felix
 */

#ifndef GYM_H_
#define GYM_H_

#include <QObject>
#include <DataCenter.h>
#include "enumDeclarations.h"


#include "json.hpp"
// for convenience
using json = nlohmann::json;


class DataCenter;


class Gym: public QObject {

	Q_OBJECT

public:
	Gym(QObject* parent = 0);
	virtual ~Gym();

private:
	static DataCenter *dc;


public slots:
	void setPlaneState(json j_data, int requestId);
	void setGymControl(const char* key, double actionValue);

signals:
	void sigSendXPDataRef(const char* dataRefName, double value);

};

#endif /* GYM_H_ */
