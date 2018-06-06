/*
 * SockServer.h
 *
 *  Created on: 30.05.2018
 *      Author: eckstein
 */

#ifndef SOCKSERVER_H_
#define SOCKSERVER_H_

#include <QObject>
#include <string>
#include <thread>

#include "json.hpp"
// for convenience
using json = nlohmann::json;

class SockServer: public QObject {
	Q_OBJECT

private:
	SockServer(); // private so it cannot be called
	SockServer(SockServer const &) :
			QObject(0) {
	} // private so it cannot be called

	SockServer & operator=(SockServer const &) {
		abort();
	}

public:
	virtual ~SockServer();

	static SockServer * getInstance() {
		if (!instance) {
			instance = new SockServer();
		}
		return instance;
	}

	void setDebug(bool _debug) {
		debug = _debug;
	}

private slots:

public slots:
	void socketSendData(std::string msgType, json data);

signals:
	void sigDispatchSockMessage(json request);

protected:
	static const std::string socket_path;
	bool quitFlag = false;
	bool isRunning = false;
	bool debug = false;

	static SockServer *instance;

private:
	void IpcListener(void);
	int connectedSocket = 0;
	void dispatchSockMessage(json receivedJson);

};

#endif /* SOCKSERVER_H_ */
