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
// TODO check portability for Windows,
// see https://stackoverflow.com/questions/2952733/using-sys-socket-h-functions-on-windows
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
	json j = { { "type", msgType }, { "requestId", requestId }, { "data", data } };
	//XXX The node.js module needs a "\f" at the end of the serialized JSON-Message :-)
	if (debug) {
		cout << "sending via socket: \n" << j.dump(4) << endl;
	}
	string msgString = j.dump() + "\f";
	unsigned int msgSize = msgString.length();
	std::set<int> socketsToRemove;
	if (!connectedSockets.empty()) {
		// wenn noch kein client auf dem Socket verbunden ist, bleibt der connectedSocket == 0
		// Das würde dann nach stdout gehen, das soll es nicht.

		for (auto connectedSocket : connectedSockets){
			unsigned int bytesTransferred;
			for (bytesTransferred = 0; bytesTransferred < msgSize;) {
				int nbytes;
				try {
					nbytes = TEMP_FAILURE_RETRY (write(connectedSocket, msgString.c_str(), msgSize));
				} catch (const std::exception& e) {
					std::cout << "Reception Error while getting data from Socket\n";
					std::cout << e.what();
					std::cout << "Some Error occurred on Socket:" << connectedSocket <<": "<<  strerror(errno) << std::endl;
					//Erasing from an active iterator is problematic, so I better create a list of sockets to remove and remove them after iteration //see https://stackoverflow.com/a/2874533/2682209
					socketsToRemove.insert(connectedSocket);
					break;
				}
				if(nbytes == -1){
					//some error occurred
					std::cout << "Some Error occurred on Socket:" << connectedSocket <<": "<<  strerror(errno) << std::endl;
					//Erasing from an active iterator is problematic, so I better create a list of sockets to remove and remove them after iteration //see https://stackoverflow.com/a/2874533/2682209
					socketsToRemove.insert(connectedSocket);
					break;
				}
				bytesTransferred += nbytes;
			}
		}
	}
	if(!socketsToRemove.empty()){
		for (auto rmv : socketsToRemove){
			connectedSockets.erase(rmv);
			std::cout << "Removing socket "<< rmv <<" from connection list. Please reconnect.\n";
		}
	}
}

void SockServer::listenForData(int connectedSocket){
	char buf[1024];
	int readCount;
	std::string receivedMsg;
	json receivedJson;

	std::cout <<"Starting IPCListener on connectedSocket " << connectedSocket<<".\n";

	do {
		try {
			readCount = read(connectedSocket, buf, sizeof(buf));
			receivedMsg = std::string(buf, readCount);
		} catch (const std::exception& e) {
			std::cout << "Reception Error while getting data from Socket\n";
			std::cout << e.what();
			break;
		}
//		// obviously, node adds the \0 at the end of the string which is then transferred via IPC
//		std::string receivedMsg = std::string(buf, readCount - 1);

		try {
			receivedJson = json::parse(receivedMsg);
		} catch (const std::exception& e) {
			std::cout << "Reception Error while parsing data from Socket\n";
			std::cout << e.what();
			continue;
		}

		if (debug) {
			std::cout << "read " << readCount << " bytes: " << receivedMsg
					<< std::endl;
			std::cout << receivedJson.dump(4) << endl;
		}

		emit sigDispatchSockMessage(receivedJson);

	} while (readCount > 0);

	if (readCount == -1) {
		perror("read");
		return;
	} else if (readCount == 0) {
		std::cout << "EOF - closing IPC socket " << connectedSocket <<std::endl;
		close(connectedSocket);
		//remove the socket from the list of connected sockets within the write-thread only
		//SockServer::socketSendData() which iterates over the set of connectedSockets.
		//Otherwise, the iterator of the socketSendData()-thread may become invalid
		//
//		connectedSockets.erase(connectedSocket);
//		std::cout <<"Removed "<< connectedSocket <<" from connected Sockets.\n";
//		std::cout <<"Remaining connected Sockets: "<< connectedSockets.size() <<".\n";
	}

}

void SockServer::IpcListener() {

	// the default socket for communicating via ipc
	static std::string socket_path = "/tmp/eee_AutoViewer";

	struct sockaddr_un addr;

	int socketDescriptor;

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

	int newConnectedSocket;

	while (!quitFlag) {
		if ((newConnectedSocket = accept(socketDescriptor, NULL, NULL)) == -1) {
			perror("accept error");
			continue;
		}
		connectedSockets.insert(newConnectedSocket);
		std::thread t(&SockServer::listenForData, this, newConnectedSocket);

		t.detach();

	}

	std::cout << "SockServer is going to stop!" << std::endl;

	// exit requested.
	close(socketDescriptor);
	isRunning = false;

	return;
}

