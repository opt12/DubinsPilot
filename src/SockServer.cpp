/*
 * SockServer.cpp
 *
 *  Created on: 30.05.2018
 *      Author: eckstein
 */

#include "SockServer.h"
#include <thread>
#include <string>
//this is to use the IPC sockets
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <iostream>
#include <time.h>
#include <string>

using namespace std;

// initialize statics
SockServer * SockServer::instance = NULL;
const std::string SockServer::socket_path = "/tmp/eee_AutoViewer";

SockServer::SockServer() :
		QObject(0) {

	debug = false;
	quitFlag = false;
	isRunning = false;

	std::thread t(&SockServer::IpcListener, this);

	t.detach();

}

SockServer::~SockServer() {
	quitFlag = true;
	time_t nowTime = time(NULL);

	while (isRunning && nowTime < time(NULL) - 5) {
		if (debug) {
			cerr << "waiting for SockServer to stop" << endl;
		}
	}

	if (isRunning) {
		cerr << "... SockServer failed to stop within 5 seconds." << endl;
	}
}

void SockServer::socketSendData(std::string msgType, int requestId, json data) {
	json j =
			{ { "type", msgType }, { "requestId", requestId }, { "data", data } };
	//XXX The node.js module needs a "\f" at the end of the serialized JSON-Message :-)
	if (debug) {
		cout << "sending via socket: \n" << j.dump(4) << endl;
	}
	string msgString = j.dump() + "\f";
	unsigned int msgSize = msgString.length();
	if (connectedSocket != 0) {
		// wenn noch kein client auf dem Socket verbunden ist, bleibt der connectedSocket == 0
		// Das würde dann nach stdout gehen, das soll es nicht.
		for (unsigned int bytesTransferred = 0; bytesTransferred < msgSize;) {
			bytesTransferred += write(connectedSocket, msgString.c_str(),
					msgSize);
		}
	}
}

void SockServer::IpcListener() {

	// the default socket for communicating via ipc
	static std::string socket_path = "/tmp/eee_AutoViewer";

	struct sockaddr_un addr;
	char buf[1024];

	json receivedJson;

	int socketDescriptor, readCount;

	if ((socketDescriptor = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
		perror("socket error");
		return;
	}

	if (socket_path.length() > 91) {
		perror(
				"pathname too long for some implementations (limit is 92 including \\0 at the end");
		return;
	}
	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, socket_path.c_str(), socket_path.length());
	unlink(socket_path.c_str()); //this is adapted from an example form https://stackoverflow.com/a/39848936

	if (bind(socketDescriptor, (struct sockaddr*) &addr, sizeof(addr)) == -1) {
		perror("bind error");
		return;
	}

	std::cout << "Listener Thread: bound ipc to \"" << addr.sun_path
			<< std::endl;

	if (listen(socketDescriptor, 5) == -1) {
		perror("listen error");
		return;
	}

	isRunning = true;

	std::cout << "IPC Listener: Waiting for data: " << std::endl;

	while (!quitFlag) {
		if ((connectedSocket = accept(socketDescriptor, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}

		while ((readCount = read(connectedSocket, buf, sizeof(buf))) > 0) {
			std::string receivedMsg = std::string(buf, readCount - 1);
			//obviously, node adds the \0 at the end of the string which is then transferred via IPC

			try {
				receivedJson = json::parse(receivedMsg);
			} catch (const std::exception& e) {
				std::cout << "Reception Error while getting data from Socket\n";
				std::cout << e.what();
				continue;
			}

			if (debug) {
				std::cout << "read " << readCount << " bytes: " << receivedMsg
						<< std::endl;
				std::cout << receivedJson.dump(4) << endl;
			}

			emit sigDispatchSockMessage(receivedJson);

		}

		if (readCount == -1) {
			perror("read");
			return;
		} else if (readCount == 0) {
			printf("EOF\n");
			close(connectedSocket);
			return;	//TODO we don't want to exit in real life
		}
	}

	std::cout << "SockServer is going to stop!" << std::endl;

	// exit requested.
	close(socketDescriptor);
	isRunning = false;

	return;
}

