///////////////////////////////////////////////////////////////////////////////
#include <opencv2\opencv.hpp> 
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "server.h"
#include "NL_process.h"
#include <vector>
#include <algorithm>
using namespace cv;

//#include "nl.h"

///////////////////////////////////////////////////////////////////////////////
u8 lrsmap[NumOfChannels];
deque<Neuron*> neurons;
deque<int>* direct_neurons = new deque<int>[NumOfChannels];
deque<int>* fire_n = new deque<int>[FrameHeight/10];
#define saliency_th 0.001

///////////////////////////////////////////////////////////////////////////////

int main()
{	// step 1: load neuron information, mainly for templates and lrs construct
	printf("Loading neurons... ");
	neurons = loadNeurons();
	printf("done.\n");

	// step 2: saliency detection
	// find each neuron's important electrode
	// need to first process the templates into figures

	/*
	printf("Saliency detection... ");
	auto startTime = std::chrono::high_resolution_clock::now();
	feedback(neurons,saliency_th);
	auto endTime = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	printf("done.\n");
	std::cout << "saliency time: " << duration.count() << " ms" << std::endl;*/

	// step 3: find each electrode's important neurons
	direct_neurons = firing_neuron(neurons);
	

	// step 4: begin loading training information 
	LrsHandler* lrsHandler = NULL;

	Kernel* kernel = new Kernel("LrsMap.txt", "HdsReq.txt");
	Server* server = new Server(kernel);

	server->stream();

	delete server;
	delete kernel;
	return 0;
}

///////////////////////////////////////////////////////////////////////////////

void Server::stream()
{
	if (!handshake())
		return;

	int lrs_counter = 0;
	int hds_counter = 0;
	auto start = high_resolution_clock::now();

	while (true) {
		char* msg = recvMsgFromTcp(tcp_sock_headstage);
		if (!msg)
			break;
		bool ok = false;
		switch (getMsgCode(msg)) {
 		case MSG_LRS:  ok = lrsHandler(msg); lrs_counter++; break;
		case MSG_HDS:  ok = hdsHandler(msg); hds_counter++; break;
		case MSG_HALO: ok = true;                           break;
		default:
			assert(false);
		}
		if (!ok)
			break;
		auto end = high_resolution_clock::now();
		auto dur = duration_cast<microseconds>(end - start);
		double t = dur.count() / 1e6;
		double lrs_fps = lrs_counter / t;
		double hds_fps = hds_counter / t;
		printf("t=%.6f, fps=%f, %f\n", t, lrs_fps, hds_fps);
	}

}

///////////////////////////////////////////////////////////////////////////////

bool Server::handshake() 
{
	if (estTcpConn()
	 && training()
	 && setupFrameBuffer()
	 && setupLrsMap()) {
		printf("handshake done\n");
		return true;
	}
	closesocket(tcp_sock_server);
	WSACleanup();
	printf("handshake failed\n");
	return false;
}

bool Server::estUdpConn() 
{
	WSADATA wsaData;
	assert(WSAStartup(MAKEWORD(2, 2), &wsaData) == 0);
	udp_sock_tx = socket(AF_INET, SOCK_DGRAM, 0);
	udp_sock_rx = socket(AF_INET, SOCK_DGRAM, 0);
	if (udp_sock_tx < 0 || udp_sock_rx < 0) {
		printf("estUdpConn: create socket err: %d\n", errno);
		return false;
	}

	int addrlen = sizeof(SOCKADDR_IN);
	SOCKADDR_IN sa = {0};
	sa.sin_family = AF_INET;
	sa.sin_addr.S_un.S_addr = INADDR_ANY;
	sa.sin_port = htons(UDP_PORT);
	if (::bind(udp_sock_rx, (SOCKADDR*)&sa, addrlen) == SOCKET_ERROR) {
		printf("estUdpConn: rx socket bind err: %d\n", errno);
		return false;
	}

	memset(&udp_addr_headstage, 0, addrlen);
	udp_addr_headstage.sin_family = AF_INET;
	udp_addr_headstage.sin_addr.s_addr = inet_addr(HEADSTAGE_IP);
	udp_addr_headstage.sin_port = htons(UDP_PORT);

	printf("estUdpConn: udp sockets created\n");
	return true;
}

bool Server::estTcpConn() 
{
	WSADATA wsaData;
	assert(WSAStartup(MAKEWORD(2, 2), &wsaData) == 0);
	tcp_sock_server = socket(AF_INET, SOCK_STREAM, 0);
	if (tcp_sock_server < 0) {
		printf("estTcpConn: create socket err: %d\n", errno);
		return false;
	}
	printf("estTcpConn: socket created\n");

	int err = 0;
	int val = 1;
	err = setsockopt(tcp_sock_server, IPPROTO_TCP,TCP_NODELAY, (char*)&val,
	                                                          sizeof(val));
	if (err == -1) {
		printf("estTcpConn: failed setting TCP_NODELAY\n");
		return false;
	}

	printf("estTcpConn: socket created on %s:%d\n", SERVER_IP, TCP_PORT);

	SOCKADDR_IN sa = {0};
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = inet_addr(SERVER_IP);
	sa.sin_port = htons(TCP_PORT);
	if (::bind(tcp_sock_server, (SOCKADDR*)&sa, sizeof(SOCKADDR)) < 0) {
		printf("estTcpConn: bind err: %d\n", errno);
		return false;
	}
	if (listen(tcp_sock_server, 10) < 0) {
		printf("estTcpConn: listen err: %d\n", errno);
		return false;
	}

	printf("estTcpConn: waiting for client connection\n");

	SOCKADDR_IN ca = {0};
	int addrlen = sizeof(SOCKADDR);
	tcp_sock_headstage = accept(tcp_sock_server, (SOCKADDR*)&ca, 
	                                             &addrlen);
	if (tcp_sock_headstage < 0) {
		printf("estTcpConn: accept connection err: %d\n", errno);
		return false;
	}

	printf("estTcpConn: successfully connected\n");
	return true;
}

bool Server::training() 
{
	const int n_tr_frames = TrainingPeriod * FrameRate;
	auto start = high_resolution_clock::now();
	for (int i = 0; i < n_tr_frames; i++) {
		char* msg = recvMsgFromTcp(tcp_sock_headstage);
		if (!msg) {
			printf("training failed\n");
			return false;
		}
		assert(getMsgCode(msg) == MSG_FRAME);
		auto end = high_resolution_clock::now();
		auto dur = duration_cast<microseconds>(end - start);
		double t = dur.count() / 1e6;
		double fps = (i + 1) / t;
		double bps = 8 * fps * FrameSize;
		cout << "t=" << t << "s, ";
		cout << "fps=" << fps << ", ";
		cout << "bps=" << bps / 1e6 << "e6";
		cout << "\n";
		delete[] msg;
	}
	printf("training done\n");
	return true;
}

bool Server::setupFrameBuffer()
{
	u8 buffer_step[NumOfChannels] = {0};
	kernel->getBufferStep(buffer_step);
	int len = MsgHdrLen + NumOfChannels;
	char* msg = new char[len];
	setMsgHdr(msg, MSG_BUF_STEP, 0, len);
	memcpy(msg + MsgHdrLen, buffer_step, NumOfChannels);
	bool ok = sendMsgToTcp(tcp_sock_headstage, msg);
	return ok;
}

bool Server::setupLrsMap() 
{
	if (!kernel->computeLrsMap(lrs_map)) {
		cout << "comptue lrs map failed\n";
		return false;
	}
	cout << "computing lrs map done\n";
	int len = MsgHdrLen + NumOfChannels;
	char* msg = new char[len];
	assert(msg);
	setMsgHdr(msg, MSG_LRS_MAP, 0, len);
	memcpy(msg + MsgHdrLen, lrs_map, NumOfChannels);
	bool ok = sendMsgToTcp(tcp_sock_headstage, msg);
	return ok;
}

///////////////////////////////////////////////////////////////////////////////
int temp = -1;

bool Server::lrsHandler(char* msg)
{   
	//auto start_time = chrono::high_resolution_clock::now();
	Frame* f = new Frame(getMsgFid(msg));
	frame_q.push_back(f);
	//ofstream file("drop_NL.txt", std::ios::app);
	//file << f->fid << std::endl;
	//file.close();
	char* pld = msg + MsgHdrLen + SamplesHdrLen;
	int n_samples_recved = samplesHdr(msg)->n_samples;
	int n_samples_wrote = 0;
	for (int ch = 0; ch < NumOfChannels; ch++) {
		if (lrs_map[ch] > 0) {
			for (int t = 0; t < FrameHeight; t += lrs_map[ch]) {
				assert(n_samples_wrote < n_samples_recved);
				writeSample((u32*)f->sample[t], ch, 
					readSample((u32*)pld, n_samples_wrote++));
			}
		}
	}
	assert(n_samples_recved == n_samples_wrote);
	delete[] msg;

	// compute and sort hds req
	if (!kernel->computeHdsReq(f,neurons)) {
		printf("lrsHandler: compute hds req failed\n");
		return false;
	}

	// assemble hds req
	char* hds_req = new char[HsRxBufSize];
	char* req_ptr = hds_req + MsgHdrLen + HdsReqHdrLen;
	int hds_req_len = MsgHdrLen + HdsReqHdrLen;
	int n_req_samples = 0;
	int n_reqs = 0;

	for (int i = 0; i < f->hds_req.size(); ) {
		HdsReq* r = &f->hds_req[i];
		int n = 0; // samples of this request
		int ts = max(r->sample_idx, 0);
		int te = min(r->sample_idx + SpikeLen, FrameHeight);
		int lrs_sampling_step = lrs_map[r->channel];
		if (lrs_sampling_step == 0) {
			n += (int)ceil((float)(te-ts) / r->step);
		} else {
			for (int t = ts; t < te; t += r->step) {
				if (t % lrs_sampling_step != 0)
					n++;
			}
		}
		if (n == 0) {
			f->hds_req.erase(f->hds_req.begin()+i);
			continue;
		}
		if (hds_req_len + HdsReqCodeLen > HsRxBufSize
		 || n_req_samples + n > MaxSamplesPerMsg) {
			setMsgHdr(hds_req, MSG_HDS_REQ, f->fid, hds_req_len);
			hdsReqHdr(hds_req)->n_req_samples = n_req_samples;
			hdsReqHdr(hds_req)->n_reqs = n_reqs;
			hdsReqHdr(hds_req)->more = true;
			if (!sendMsgToTcp(tcp_sock_headstage, hds_req))
				return false;
			hds_req = new char[HsRxBufSize];
			req_ptr = hds_req + MsgHdrLen + HdsReqHdrLen;
			hds_req_len = MsgHdrLen + HdsReqHdrLen;
			n_req_samples = 0;
			n_reqs = 0;
		}
		encHdsReq(req_ptr, r);
		req_ptr += HdsReqCodeLen;
		hds_req_len += HdsReqCodeLen;
		n_req_samples += n;
		n_reqs++;
		i++;
	}
	setMsgHdr(hds_req, MSG_HDS_REQ, f->fid, hds_req_len);
	hdsReqHdr(hds_req)->n_req_samples = n_req_samples;
	hdsReqHdr(hds_req)->n_reqs = n_reqs;
	hdsReqHdr(hds_req)->more = false;
	//auto end_time = chrono::high_resolution_clock::now();
	//auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);
	//cout << "handler_lrs time: " << duration.count() << " us" << std::endl; // 87 us
	if (!sendMsgToTcp(tcp_sock_headstage, hds_req))
		return false;
	return true;
}

bool Server::hdsHandler(char* msg)
{
	//auto start_time = chrono::high_resolution_clock::now();

	assert(!frame_q.empty() && frame_q[0]->fid == getMsgFid(msg));
	Frame* f = frame_q[0];
	// copy hd samples
	char* pld = msg + MsgHdrLen + SamplesHdrLen;
	int n_samples_recved = samplesHdr(msg)->n_samples;
	int n_samples_wrote = 0;
	while (n_samples_wrote < n_samples_recved) {
		assert(!f->hds_req.empty());
		HdsReq* r = &f->hds_req[0];
		int ts = max(r->sample_idx, 0);
		int te = min(r->sample_idx + SpikeLen, FrameHeight);
		int lrs_sampling_step = lrs_map[r->channel];
		if (lrs_sampling_step == 0) {
			for (int t = ts; t < te; t += r->step) {
				assert(n_samples_wrote < n_samples_recved);
				writeSample((u32*)f->sample[t], r->channel,
					readSample((u32*)pld, n_samples_wrote++));
			}
		} else {
			for (int t = ts; t < te; t += r->step) {
				if (t % lrs_sampling_step == 0)
					continue;
				assert(n_samples_wrote < n_samples_recved);
				writeSample((u32*)f->sample[t], r->channel,
					readSample((u32*)pld, n_samples_wrote++));
			}
		}
		f->hds_req.pop_front();
	}
	delete[] msg;
	if (f->hds_req.empty()) {
		frame_q.pop_front();
		delete f;
	}
	//auto end_time = chrono::high_resolution_clock::now();
	//auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);
	//cout << "handler_hds time: " << duration.count() << " us" << std::endl; // 87 us
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool Server::sendBytesToTcp(SOCKET sock, char* p, int len)
{
	char* p_cur = p;
	char* p_end = p+len;
	while (p_cur < p_end) {
		int n = send(sock, p_cur, p_end-p_cur, 0);
		if (n <= 0) {
			printf("sendBytesToTcp err: %d\n", errno);
			return false;
		}
		p_cur += n;
	}
	return true;
}

bool Server::sendMsgToTcp(SOCKET sock, char* msg)
{
	if (!sendBytesToTcp(sock, msg, getMsgLen(msg))) {
		printf("[%08d] tcp send %s failed\n", getMsgFid(msg),
		                                      getMsgCodeStr(msg));
		delete[] msg;
		return false;
	}
	printf("[%08d] tcp sent %s of %d bytes\n", getMsgFid(msg),
	                                           getMsgCodeStr(msg),
	                                           getMsgLen(msg));
	delete[] msg;
	return true;
}

bool Server::sendBytesToUdp(SOCKET sock, sockaddr_in* ra, char* p, int len)
{
	char* p_cur = p;
	char* p_end = p + len;
	while (p_cur < p_end) {
		int n = sendto(sock, p_cur, p_end - p_cur, 0, (sockaddr*)ra,
		                                          sizeof(sockaddr));
		if (n <= 0)
			return false;
		p_cur += n;
	}
	return true;
}

bool Server::sendMsgToUdp(SOCKET sock, sockaddr_in* ra, char* msg)
{
	if (!sendBytesToUdp(sock, ra, msg, getMsgLen(msg))) {
		printf("[%08d] udp send %s failed\n", getMsgFid(msg),
		                                      getMsgCodeStr(msg));
		return false;
	}
	printf("[%08d] udp sent %s of %d bytes\n", getMsgFid(msg),
	                                           getMsgCodeStr(msg),
	                                           getMsgLen(msg));
	return true;
}

bool Server::recvBytesFromTcp(SOCKET sock, char* p, int len)
{
	char* p_end = p+len;
	char* p_cur = p;
	while (p_cur < p_end) {
		int n = recv(sock, p_cur, p_end-p_cur, 0);
		if (n <= 0) {
			printf("recvBytesFromTcp err: %d\n", errno);
			return false;
		}
		p_cur += n;
	}
	return true;
}

char* Server::recvMsgFromTcp(SOCKET sock)
{
	char hdr[MsgHdrLen] = {0};
	if (!recvBytesFromTcp(sock, hdr, MsgHdrLen)) {
		printf("recv msg hdr failed\n");
		return NULL;
	}
	u32 len = getMsgLen(hdr);
	u8 code = getMsgCode(hdr);
	u32 fid = getMsgFid(hdr);
	char* msg = new char[len];
	assert(msg);
	setMsgHdr(msg, code, fid, len);
	if (!recvBytesFromTcp(sock, msg+MsgHdrLen, len-MsgHdrLen)) {
		printf("recv msg pld failed\n");
		delete[] msg;
		return NULL;
	}
	printf("[%08ld] recved %s of %d bytes\n", getMsgFid(msg),
	                                          getMsgCodeStr(msg),
	                                          getMsgLen(msg));
	return msg;

}

bool Server::recvBytesFromUdp(SOCKET sock, char* p, int len)
{
	sockaddr sa = {0};
	int addrlen = sizeof(sa);
	char* p_end = p + len;
	char* p_cur = p;
	while (p_cur < p_end) {
		int n = recvfrom(sock, p_cur, p_end-p_cur, 0, &sa, &addrlen);
		if (n <= 0) {
			printf("recvBytesFromUdp err: %d\n", errno);
			return false;
		}
		p_cur += n;
	}
	return true;
}

char* Server::recvMsgFromUdp(SOCKET sock)
{
	char hdr[MsgHdrLen] = { 0 };
	if (!recvBytesFromUdp(sock, hdr, MsgHdrLen)) {
		printf("udp recv msg hdr failed\n");
		return NULL;
	}
	u32 len = getMsgLen(hdr);
	u8 code = getMsgCode(hdr);
	u32 fid = getMsgFid(hdr);
	char* msg = new char[len];
	assert(msg);
	setMsgHdr(msg, code, fid, len);
	if (!recvBytesFromUdp(sock, msg+MsgHdrLen, len-MsgHdrLen)) {
		printf("udp recv msg pld failed\n");
		delete[] msg;
		return NULL;
	}
	printf("[%08ld] udp recved %s of %d bytes\n", getMsgFid(msg),
		                                      getMsgCodeStr(msg),
		                                      getMsgLen(msg));
	return msg;
}

///////////////////////////////////////////////////////////////////////////////

Kernel::Kernel(const char* lrs_map_dir, const char* hds_req_dir)
{
	printf("loading kernel traces ...\n");

	assert(loadLrsMap(lrs_map_dir));
	assert(loadHdsReq(hds_req_dir));

	printf("computing buffer steps ...\n");
	for (LrsMapLog i : lrs_map_log) {
		buffer_step[i.ch] = i.step;
	}
	for (HdsReqLog i : hds_req_log) {
		if (buffer_step[i.ch] == 0 || i.step < buffer_step[i.ch])
			buffer_step[i.ch] = i.step;
	}
	int unused_channels = 0;
	double ram_ratio = 0;
	for (int ch = 0; ch < NumOfChannels; ch++) {
		printf("ch: %3d, buffer step: %d\n", ch, buffer_step[ch]);
		if (buffer_step[ch] == 0)
			unused_channels++;
		else
			ram_ratio += 1.0f / buffer_step[ch];
	}
	ram_ratio /= 374;//NumOfChannels;
	printf("unused channels: %d\n", unused_channels);
	printf("RAM ratio: %f\n", ram_ratio);

	//printf("continue?\n");
	//getchar();
}

bool Kernel::loadLrsMap(const char* filename)
{
	printf("loading lrs map from %s ... ", filename);
	ifstream fin(filename);
	if (!fin.is_open()) {
		printf("\nloadLrsMap err: cannot open file\n");
		return false;
	}
	LrsMapLog tmp = {0};
	while (fin >> tmp.ch >> tmp.step) {
		if (tmp.ch < NumOfChannels)
			lrs_map_log.push_back(tmp);
	}
	fin.close();
	printf("done\n");
	return true;
}

bool Kernel::loadHdsReq(const char* filename)
{
	printf("loading hds req from %s ...", filename);
	ifstream fin(filename);
	if (!fin.is_open()) {
		printf("\nloadHdsReq err: cannot open file\n");
		return false;
	}
	HdsReqLog tmp = {0};
	while (fin >> tmp.t >> tmp.ch >> tmp.step) {
		if (tmp.ch < NumOfChannels) {
			hds_req_log.push_back(tmp);
		}
	}
	fin.close();
	printf(" done\n");
	printf("sorting hds req ...");
	int N = hds_req_log.size();
	bool sorted = false;
	for (int i = 0; i < N && !sorted; i++) {
		sorted = true;
		for (int j = N - 1; j > 0; j--) {
			if (hds_req_log[j].t < hds_req_log[j-1].t) {
				HdsReqLog tmp = hds_req_log[j-1];
				hds_req_log[j-1] = hds_req_log[j];
				hds_req_log[j] = tmp;
				sorted = false;
			}
		}
	}
	printf(" done.\n");
	double trace_len = (double)(hds_req_log.back().t) / SampleRate;
	printf("loaded %ld hds req of %fs\n", hds_req_log.size(), 
		                              trace_len);
	return true;
}

void Kernel::getBufferStep(u8* buffer_step)
{
	memcpy(buffer_step, this->buffer_step, NumOfChannels);
}

bool Kernel::computeLrsMap(u8* lrs_map)
{
	printf("computing lrs map ...\n");
	memset(lrs_map, 0, NumOfChannels);
	for (LrsMapLog i : lrs_map_log)
		lrs_map[i.ch] = i.step;

	printf("adjusting lrs samplng steps based on buffer steps ...\n");
	double prev_lrs_ratio = 0;
	double adjusted_lrs_ratio = 0;
	for (int ch = 0; ch < NumOfChannels; ch++) {
		if (lrs_map[ch] == 0)
			continue;
		int adjusted_step = lrs_map[ch];
		adjusted_step /= buffer_step[ch];
		adjusted_step *= buffer_step[ch];
		printf("ch:%3d, prev lrs step:%2d adjusted step:%2d\n",
			ch, lrs_map[ch], adjusted_step);
		prev_lrs_ratio += 1.0f / lrs_map[ch];
		adjusted_lrs_ratio += 1.0f / adjusted_step;
		lrs_map[ch] = adjusted_step;
		lrsmap[ch] = lrs_map[ch];
	}
	printf("prev lrs ratio: %f\n", prev_lrs_ratio / NumOfChannels);
	printf("adjusted ratio: %f\n", adjusted_lrs_ratio / NumOfChannels);
	printf("lrs inflation : %f\n", adjusted_lrs_ratio / prev_lrs_ratio);

	return true;
}

#define th_positive 80
#define th_negative -80
#define Det_Window  10

bool Kernel::computeHdsReq(Frame* f, deque<Neuron*>& neurons)
{	
	//auto start_time = std::chrono::high_resolution_clock::now();
	deque<HdsReqLog> hds_log;

	// step 1: read the raw data to change the value of f->sample
	FILE* fp;
	long fid = f->fid;
	i16 SamplePerFrame = FrameHeight * NumOfChannels;
	i16* frame_buf = new i16[SamplePerFrame];
	fp = fopen("./data/human_60s.dat", "r"); // original data in binary 
	fseek(fp, fid * SamplePerFrame * 2, SEEK_SET); // set the offset to control the begin of the reader
	memset(frame_buf,0, SamplePerFrame);
	fread(frame_buf, 2, SamplePerFrame, fp);
	fclose(fp);

	// step 2: potential firing neurons detection
	for (int i = 0;i < FrameHeight;i = i + Det_Window) {
		// check power in lrs electrodes
		fire_n[i / Det_Window].clear(); // potential firing neurons
		bool* pwr_res = new bool[NumOfChannels] {0};
		for (int j = 0;j < NumOfChannels;j++) {
			if (lrsmap[j] > 0) {
				// power detection trigger
				for (int m = i;m < i + Det_Window;m = m + lrsmap[j]) {
					//i16 temp = readSample((u32*)f->sample[i + m + lrsmap[j] - i % lrsmap[j] - 1], j);
					int temp = frame_buf[m * NumOfChannels + j];
					//cout << temp << endl;
					if ((temp > th_positive) || (temp < th_negative)) {
						pwr_res[j] = 1;
						for (int k = 0;k < direct_neurons[j].size();k++) {
							fire_n[i/ Det_Window].push_back(direct_neurons[j][k]);
						}
						break;
					}
				}
			}
		}
	}
		//auto end_time = std::chrono::high_resolution_clock::now();
		//auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		//cout << "Neuron detection time: " << duration.count() << " us" << std::endl; // 87 us

		//auto start_time1 = std::chrono::high_resolution_clock::now();

		
		// step 3: derive hds_req_log for neurons with enough scores
		for (int q=0;q<FrameHeight/Det_Window;q++){
			if (fire_n[q].size()) {
				sort(fire_n[q].begin(), fire_n[q].end());
				for (int i = 0;i < fire_n[q].size();i++) {
	//#pragma omp parallel for schedule(static) private(m)
					for (int m = 0;m < neurons[fire_n[q][i]]->firing_electrode.size();m++) {
						int k = neurons[fire_n[q][i]]->firing_electrode[m];
						//int k = 0;
						HdsReqLog x;
						x.step = 2;
						// cout<<x.Samp_Inter<<endl;
						x.ch = k;
						x.t = i + Det_Window / 2 + fid * FrameHeight;
						hds_log.push_back(x);
					}
				}
			}
		}
		

	delete[] frame_buf;

	//auto end_time1 = std::chrono::high_resolution_clock::now();
	//duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time1 - start_time1);
	//cout << "Deriving feedback time: " << duration.count() << " us" << std::endl;
	

	// replace hds_req_log with hds_log
	// hds_req_log = hds_log;
	
	// start_time1 = std::chrono::high_resolution_clock::now();
	if (hds_req_log.empty())
		return false;
	long fs = f->fid * FrameHeight; // frame start
	long fe = fs     + FrameHeight; // frame end
	// pop obsolete req logs
	while (!hds_req_log.empty() && hds_req_log[0].t + SpikeLen < fs) {
		hds_req_log.pop_front();
	}
	f->hds_req.clear();
	for (HdsReqLog i : hds_req_log) {
		if (i.t >= fe) break;
		HdsReq tmp = { 0 };
		tmp.sample_idx = i.t - fs;
		tmp.channel = i.ch;
		tmp.step = i.step;
		// adjust bsaed on buffer steps
		tmp.sample_idx /= buffer_step[i.ch];
		tmp.sample_idx *= buffer_step[i.ch];
		tmp.step /= buffer_step[i.ch];
		tmp.step *= buffer_step[i.ch];
		f->hds_req.push_back(tmp);
	}
	
	printf("adjusting hds req steps based on buffer steps ...\n");
	double prev_hds_ratio = 0;
	double adjusted_hds_ratio = 0;
	for (HdsReqLog i : hds_req_log) {
		int adjusted_step = buffer_step[i.ch];
		while (adjusted_step + buffer_step[i.ch] <= i.step) {
			adjusted_step += buffer_step[i.ch];
		}
		prev_hds_ratio += 1.0f / i.step;
		adjusted_hds_ratio += 1.0f / adjusted_step;
		i.step = adjusted_step;
	}
	printf("hds inflation : %f\n", adjusted_hds_ratio / prev_hds_ratio);
	

	//end_time1 = std::chrono::high_resolution_clock::now();
	//duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time1 - start_time1);
	//cout << "Processing time: " << duration.count() << " us" << std::endl;
	return true;
}