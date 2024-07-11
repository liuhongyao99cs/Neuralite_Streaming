///////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <thread>
#include <winsock.h>
#include <cstdio>
#include <cstdint>
#include <cassert>
#include <chrono>
#include <algorithm>
#include <deque>
#include <cmath>

#include "neuralite.h"
#include "NL_process.h"

#pragma comment (lib, "ws2_32.lib")

using namespace std;
using namespace chrono;

///////////////////////////////////////////////////////////////////////

class Frame {
public:
	u16 fid = 0;
	char* sample[FrameHeight];
	deque<HdsReq> hds_req;
	Frame(u16 fid) {
		this->fid = fid;
		for (int t = 0; t < FrameHeight; t++) {
			assert(sample[t] = new char[FrameWidth]);
			memset(sample[t], 0, FrameWidth);
		}
	};
	~Frame() {
		for (int t = 0; t < FrameHeight; t++)
			delete[] sample[t];
	};
};

typedef struct { int ch, step; } LrsMapLog;
typedef struct { int ch, step; long t; } HdsReqLog;

class Kernel {
private:
	deque<LrsMapLog> lrs_map_log;
	deque<HdsReqLog> hds_req_log;
	deque<Neuron*> neurons;
	u8 buffer_step[NumOfChannels];
	bool loadLrsMap(const char*);
	bool loadHdsReq(const char*);
public:
	Kernel(const char*, const char*);
	bool computeLrsMap(u8*);
	bool computeHdsReq(Frame*, deque<Neuron*>& neurons);
	void getBufferStep(u8*);
};

class Server {
protected:
	SOCKET tcp_sock_server = 0;
	SOCKET tcp_sock_headstage = 0;
	SOCKADDR_IN udp_addr_headstage;
	SOCKET udp_sock_tx = 0;
	SOCKET udp_sock_rx = 0;

	Kernel* kernel;
	u8 lrs_map[NumOfChannels] = { 0 };
	deque<Frame*> frame_q;

	bool  estUdpConn();
	bool  estTcpConn();
	bool  training();
	bool  setupFrameBuffer();
	bool  setupLrsMap();
	bool  handshake();

	bool  lrsHandler(char*);
	bool  hdsHandler(char*);

	bool  sendBytesToTcp(SOCKET, char*, int);
	bool  sendMsgToTcp(SOCKET, char*);
	bool  sendBytesToUdp(SOCKET, sockaddr_in*, char*, int);
	bool  sendMsgToUdp(SOCKET, sockaddr_in*, char*);
	bool  recvBytesFromTcp(SOCKET, char*, int);
	char* recvMsgFromTcp(SOCKET);
	bool  recvBytesFromUdp(SOCKET, char*, int);
	char* recvMsgFromUdp(SOCKET);

public:
	Server(Kernel* k) : kernel(k) {};
	~Server() { WSACleanup(); };

	void stream();
};

deque<int>* firing_neuron();