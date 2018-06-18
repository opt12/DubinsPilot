/*
 * RequestDispatcher.h
 *
 *  Created on: 01.06.2018
 *      Author: eckstein
 */

#ifndef REQUESTDISPATCHER_H_
#define REQUESTDISPATCHER_H_

#include <QObject>

#include "json.hpp"
// for convenience
using json = nlohmann::json;

class RequestDispatcher: public QObject {
	Q_OBJECT

public:
	RequestDispatcher(QObject* parent = 0);
	virtual ~RequestDispatcher();

	void setDebug(bool debug = false) {
		this->debug = debug;
	}

public slots:
	void dispatchSockMessage(json receivedJson);

signals:
	void requested_getPosition(int requestId);
	void requested_getPositionXY(int requestId);
	void requested_getOrigin(int requestId);
	void requested_getPlaneState(int requestId);

private:
	bool debug = false;
};

#endif /* REQUESTDISPATCHER_H_ */
