#pragma once
#include <opencv2\opencv.hpp> 
#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <cassert>
#include <cmath>
#include <deque>
#include <omp.h>
#include <cstring>
#include <ctime>
#include <chrono>

using namespace cv;

//#include "neuralite.h"

/////////////////////////////////////////////////////////////////////////////////////

using namespace std;

#define NumOfElectrodes  374
#define FiringRateThresh 0.07
#define DetThreshPos     +2.2
#define DetThreshNeg     -4
#define DetConstraint    0.95
#define MaxDelta         16
#define SpikeDetWindow   10
#define l_left  	 12
#define l_right		 24// total hds length is l_left + l_right + 1 (45)
#define spikelen 61
#define sampler (int)3e4

/////////////////////////////////////////////////////////////////////////////////////
// this part define training sturtures

// ==== define neuron structure for firing detection ====
typedef struct {
	float spike[NumOfChannels][spikelen];
	float U[3][NumOfChannels];
	float weight[3][NumOfChannels];
	float fire_rate;
	float pwr;
	float std;
	deque<int> firing_electrode;
} Neuron;


// ==== saliency-based feedback ====
deque<int> Saliency_Det(Mat image,float percentage) {
	cv::resize(image,image, cv::Size(374, 61));

	cv::Mat grayImage;
	cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
	//Mat grayImage = image;
	cv::Mat saliencyMap;
	cv::blur(grayImage, saliencyMap, cv::Size(3, 3));

	int totalPixels = saliencyMap.rows * saliencyMap.cols; 
	int numPixelsToExtract = static_cast<int>(percentage * totalPixels);

	cv::Mat saliencyMapFlattened = saliencyMap.reshape(1, totalPixels); 
	saliencyMapFlattened.convertTo(saliencyMapFlattened, CV_32F);

	std::deque<int> saliencyVector(saliencyMapFlattened.begin<float>(), saliencyMapFlattened.end<float>());
	std::sort(saliencyVector.begin(), saliencyVector.end(), std::greater<float>());
	std::deque<int> columnIndices;
	columnIndices.clear();

	float pixel_threshold = saliencyVector[numPixelsToExtract];
	cv::threshold(saliencyMap, saliencyMap, pixel_threshold-1, 255, cv::THRESH_BINARY);

	std::deque<int> columns;

	for (int y = 0; y < saliencyMap.rows; y++) {
		for (int x = 0; x < saliencyMap.cols; x++) {
			float pixelValue = saliencyMap.at<uchar>(y, x);
			if (pixelValue == 255) {
				columns.push_back(x);
			}
		}
	}

	std::sort(columns.begin(), columns.end());
	columns.erase(std::unique(columns.begin(), columns.end()), columns.end());
	/*for (int columnIndex : columns) {
		std::cout << columnIndex << " ";
	}
	std::cout << std::endl;
	*/
	return columns;
}

void feedback(deque <Neuron*>& neurons, float th) {
	for (int i = 0; i < neurons.size();i++)
	{	// method 1: dirctly transform array into image for detection
		//Mat B = convertToGrayImage(neurons[i]->spike);
		//columns[i] = Saliency_Det(B, 0.0022);

		// method 2: read the image of templates pre-processed by python
		deque <int> columns;
		std::string filename = "D:/Jupyter/NeuraLite/design/template_img/image" + std::to_string(i) + ".jpg";
		cv::Mat image = cv::imread(filename);
		columns = Saliency_Det(image, th);
		neurons[i]->firing_electrode.clear();
		//electrode that contains neuron feature
		for (int k = 0;k < columns.size();k++) {
			neurons[i]->firing_electrode.push_back(columns[k]);
		}
	}
}

deque<int>* firing_neuron(deque <Neuron*>& neurons) {
	deque<int>* direct_neurons = new deque<int>[NumOfChannels];
	for (int i = 0;i < neurons.size();i++) {
		int k = neurons[i]->firing_electrode.size();
		for (int j = 0;j < k;j++) {
			direct_neurons[neurons[i]->firing_electrode[j]].push_back(i);
		}
	}
	return direct_neurons;
}

cv::Mat convertToGrayImage(float spike[][spikelen])
{
	const int M = spikelen, N = NumOfChannels;
	float data[M][N] = { 0 };
;
	for (int i = 0;i < M;i++)
	{
		for (int j = 0;j < N;j++)
		{
			data[i][j] = abs(spike[j][i] * 5);
		}
	}

	Mat B = Mat(M, N, CV_8UC1);	
	for (int i = 0;i < M;i++)
	{
		for (int j = 0;j < N;j++)
		{
				B.data[i * N + j] = data[i][j];
		}
	}

	return B;
}

cv::Mat convertToColorMap(const float spike[][spikelen])
{
	cv::Mat grayImage(spikelen, NumOfChannels, CV_32FC1);
	for (int i = 0; i < spikelen; ++i)
	{
		for (int j = 0; j < NumOfChannels; ++j)
		{
			float value = spike[j][i];
			grayImage.at<float>(i, j) = value;
		}
	}
	cv::Mat colorMap;
	cv::applyColorMap(grayImage, colorMap, cv::COLORMAP_JET);

	return colorMap;
}


// ==== define neuron structure for firing detection ====

// read training file from txt 
bool readFloat(ifstream& fs, float& dst) {
	string buf;
	if (!getline(fs, buf))
		return false;
	// how about 1e-5 to be transmitted to float value?
	string::size_type idx;
	string x = "e";
	string buf_x, buf_y;
	idx = buf.find(x);
	string m;
	float float_x;
	if (int(idx) == -1) // not exist
		dst = stof(buf);
	else
	{
		int l = buf.length() - 2;
		m = buf[l];
		//cout<<stof(m)<<" ";
		buf_x = buf.substr(0, int(idx));
		float_x = stof(buf_x);
		dst = float_x / pow(10, stof(m));
	}
	return true;
}

// load neuron information
deque<Neuron*> loadNeurons() {
	deque<Neuron*> neurons;
	ifstream fs_template("E:/NeuraLite/Server/Server/data/template.txt");
	ifstream fs_power("E:/NeuraLite/Server/Server/data/power.txt");
	ifstream fs_sigma("E:/NeuraLite/Server/Server/data/sigma.txt");
	ifstream fs_fire_rate("E:/NeuraLite/Server/Server/data/firing_rate.txt");
	ifstream fs_u("E:/NeuraLite/Server/Server/data/U.txt");
	//ifstream fs_sample("E:/NeuraLite/Server/Server/data/sample_interval.txt");
	if (!fs_template.is_open() ||
		!fs_power.is_open() ||
		!fs_sigma.is_open() ||
		!fs_fire_rate.is_open() ||
		!fs_u.is_open()
		) {
	}
	int t, e, i;
	// load the sample interval txt
	while (true) {
		Neuron* n = new Neuron;
		for (t = 0; t < spikelen; t++) {
			for (e = 0; e < NumOfChannels; e++) {
				if (!readFloat(fs_template, n->spike[e][t])) {
					delete n;
					goto end;
				}
			}
		}
		for (i = 0; i < 3; i++) {
			for (e = 0; e < NumOfChannels; e++) {
				if (!readFloat(fs_u, n->U[i][e])) {
					delete n;
					goto end;
				}
			}
		}
		for (i = 0; i < 3; i++) {
			for (e = 0; e < NumOfChannels; e++) {
				n->weight[i][e] = n->U[i][e];
			}
		}
		if (!readFloat(fs_power, n->pwr) ||
			!readFloat(fs_sigma, n->std) ||
			!readFloat(fs_fire_rate, n->fire_rate)) {
			delete n;
			goto end;
		}

		// load the important electrodes for each neuron
		for (e = 0; e < NumOfChannels; e++) {
			if ((n->U[0][e] > 0.12) || (n->U[0][e] < -0.12)
				|| (n->U[1][e] > 0.15) || (n->U[1][e] < -0.15)) {
				n->firing_electrode.push_back(e);
			}

		}

		if (n->fire_rate > FiringRateThresh)
			neurons.push_back(n);
		else
			delete n;

		// load potential neuron for electrode

	}
end:
	if (fs_template.is_open()) fs_template.close();
	if (fs_power.is_open()) fs_power.close();
	if (fs_sigma.is_open()) fs_sigma.close();
	if (fs_fire_rate.is_open()) fs_fire_rate.close();
	if (fs_u.is_open()) fs_u.close();
	return neurons;
}
void freeNeurons(deque<Neuron*> neurons) {
	deque<Neuron*>::iterator it;
	for (it = neurons.begin(); it < neurons.end(); it++)
		delete* it;
}


/// ====== compute lrs stream =========
class LrsHandler {
private:
	int NumOfNeurons;
	float*** connect_graph_ = NULL;
	deque<int> e_set_;
	deque<int> sampling_delta_;
	deque<int>* direct_neuron_ = NULL;
	float phi(float x);
	float gaussianCDF(float x, float mu, float std);
	void initConnectGraph(deque<Neuron*>& neurons);
	void freeConnectGraph();
	inline bool directConnect(int n, int e, int d);
	void computeLrsMap();
public:
	LrsHandler(deque<Neuron*>& neurons);
	~LrsHandler();
	void printConnectGraph();
	void printLrsMap();
	void initlrs(char* delta);
};
LrsHandler::LrsHandler(deque<Neuron*>& neurons) {
	NumOfNeurons = neurons.size();
	initConnectGraph(neurons);
	computeLrsMap();
}
LrsHandler::~LrsHandler() {
	if (direct_neuron_)
		delete[] direct_neuron_;
	if (connect_graph_)
		freeConnectGraph();
}
inline float LrsHandler::phi(float x) {
	return 0.5 * (1 + erf(x / sqrt(2)));
}
inline float LrsHandler::gaussianCDF(float x, float mu, float std) {
	return phi((x - mu) / std);
}
void LrsHandler::initConnectGraph(deque<Neuron*>& neurons) {
	float st, en;
	int i, n, e, t, d;
	// Monte Carlo simulation of interference
	printf("\tsimulating firing... ");
	st = omp_get_wtime();
	const int MonteCarloN = 2048;
	typedef struct {
		float pwr[NumOfElectrodes][spikelen];
		float var[NumOfElectrodes][spikelen];
	} Interfere;
	Interfere* interfere = new Interfere[MonteCarloN];
	memset(interfere, 0, MonteCarloN * sizeof(Interfere));
#pragma omp parallel for schedule(static) private(i,n,e,t)
	for (i = 0; i < MonteCarloN; i++) {
		for (n = 0; n < NumOfNeurons; n++) {
			if ((rand() % sampler) < neurons[n]->fire_rate * spikelen) {
				for (e = 0; e < NumOfElectrodes; e++) {
					float* pwr = interfere[i].pwr[e];
					float* var = interfere[i].var[e];
					float* tmp = neurons[n]->spike[e];
					for (t = rand() % spikelen; t < spikelen; t++) {
						pwr[t] += tmp[t] * neurons[n]->pwr;
						var[t] += tmp[t] * neurons[n]->std *
							tmp[t] * neurons[n]->std;
					}
				}
			}
		}
	}
	en = omp_get_wtime();
	printf("used %fs.\n", en - st);
	// allocate connect graph
	printf("\tallocating connect graph... ");
	st = omp_get_wtime();
	connect_graph_ = new float** [NumOfNeurons];
#pragma omp parallel for schedule(static) private(n,e,d)
	for (int n = 0; n < NumOfNeurons; n++) {
		connect_graph_[n] = new float* [NumOfElectrodes];
		for (int e = 0; e < NumOfElectrodes; e++) {
			connect_graph_[n][e] = new float[MaxDelta];
			for (int d = 0; d < MaxDelta; d++)
				connect_graph_[n][e][d] = 0;
		}
	}
	en = omp_get_wtime();
	printf("used %fs.\n", en - st);
	// compute edge weights for connect graph
	printf("\tcomputing graph edges... ");
	st = omp_get_wtime();
#pragma omp parallel for schedule(static) private(n,e,t,i)
	for (n = 0; n < NumOfNeurons; n++) {
		for (e = 0; e < NumOfElectrodes; e++) {
			float neg_peak = 0;
			float pos_peak = 0;
			for (t = 0; t < spikelen; t++) {
				if (neurons[n]->spike[e][t] < neg_peak)
					neg_peak = neurons[n]->spike[e][t];
				if (neurons[n]->spike[e][t] > pos_peak)
					pos_peak = neurons[n]->spike[e][t];
			}
			if ((neurons[n]->pwr - 2 * neurons[n]->std) * neg_peak > DetThreshNeg &&
				(neurons[n]->pwr + 2 * neurons[n]->std) * pos_peak < DetThreshPos) {
				// connect is too weak and negligible
				continue;
			}
			// compute the probability of detecting n by e at a specific t
			float det_pr[spikelen] = { 0 };
			for (t = 0; t < spikelen; t++) {
				// power and variance of this neuron
				float pwr0 = neurons[n]->pwr * neurons[n]->spike[e][t];
				float var0 = neurons[n]->std * neurons[n]->spike[e][t] *
					neurons[n]->std * neurons[n]->spike[e][t];
				// test connectivity for each Monte Carlo sim of interference
				float n_det = 0;
				for (i = 0; i < MonteCarloN; i++) {
					// super-impose neuron power and variance with simulated interference
					float pwr = pwr0 + interfere[i].pwr[e][t];
					float var = var0 + interfere[i].var[e][t];
					// compute gaussian CDF to estimate det_pr
					n_det += 1 - gaussianCDF(DetThreshPos, pwr, sqrt(var))
						+ gaussianCDF(DetThreshNeg, pwr, sqrt(var));
				}
				det_pr[t] = n_det / MonteCarloN;
			}
			// derive detection probability under different sampling deltas
			for (d = 0; d < MaxDelta; d++) {
				float sum = 0;
				for (t = 0; t <= d; t++)
					sum += det_pr[t];
				float max_sum = sum;
				for (t = d + 1; t < spikelen; t++) {
					sum += det_pr[t] - det_pr[t - d - 1];
					if (sum > max_sum)
						max_sum = sum;
				}
				connect_graph_[n][e][d] = max_sum / (d + 1);
			}
		}
	}
	en = omp_get_wtime();
	printf("used %fs.\n", en - st);
	delete[] interfere;
}
void LrsHandler::freeConnectGraph() {
	for (int n = 0; n < NumOfNeurons; n++) {
		for (int e = 0; e < NumOfElectrodes; e++)
			delete[] connect_graph_[n][e];
		//delete[] connect_graph_[n];
	}
	//delete[] connect_graph_;
}
inline bool LrsHandler::directConnect(int n, int e, int d) {
	return connect_graph_[n][e][d] >= DetConstraint;
}
void LrsHandler::computeLrsMap() {
	float st, en;
	int  e, d, n, i;
	// greedy cover
	printf("\tcomputing greedy cover... ");
	st = omp_get_wtime();
	deque<bool> selected(NumOfElectrodes, false);
	deque<int>  delta(NumOfElectrodes, MaxDelta);
	deque<bool> covered(NumOfElectrodes, false);
	while (true) {
		int winner_electrode = 0;
		int winner_delta = 0;
		int winner_score = 0;
#pragma omp parallel for schedule(static) private(e,d,n)
		for (e = 0; e < NumOfElectrodes; e++) {
			for (d = 0; d < delta[e]; d++) {
				int new_cover = 0;
				for (n = 0; n < NumOfNeurons; n++) {
					if (!covered[n] && directConnect(n, e, d))
						new_cover++;
				}
				if (new_cover == 0)
					break;
				else if (new_cover * d > winner_score) {
					winner_electrode = e;
					winner_score = new_cover * d;
					winner_delta = d;
				}
			}
		}
		if (winner_score > 0) {
			selected[winner_electrode] = true;
			delta[winner_electrode] = winner_delta;
			for (n = 0; n < NumOfNeurons; n++)
				if (directConnect(n, winner_electrode, winner_delta))
					covered[n] = true;
		}
		else
			break;
	}
	en = omp_get_wtime();
	printf("used %fs.\n", en - st);
	// remove redundant electrodes
	printf("\tremove redundant electrodes... ");
	st = omp_get_wtime();
	bool redundant[NumOfElectrodes] = { 0 };
	while (true) {
		for (e = 0; e < NumOfElectrodes; e++)
			redundant[e] = true;
		// mark all non-redundant electrodes
		for (n = 0; n < NumOfNeurons; n++) {
			deque<int> direct_e; // the e set that direclty connect n
			for (e = 0; e < NumOfElectrodes; e++) {
				if (selected[e] && directConnect(n, e, delta[e]))
					direct_e.push_back(e);
			}
			// e is not redundant if it's the only e that connects a neuron
			if (direct_e.size() == 1)
				redundant[direct_e.front()] = false;
		}
		// remove the first redundant selected e, if any
		bool done = true;
		for (e = 0; e < NumOfElectrodes; e++) {
			if (selected[e] && redundant[e]) {
				done = false;
				selected[e] = false;
				break;
			}
		}
		if (done)
			break;
	}
	en = omp_get_wtime();
	printf("used %fs.\n", en - st);
	// init LrsMap
	printf("\tconstruct LrsMap... ");
	st = omp_get_wtime();
	for (e = 0; e < NumOfElectrodes; e++) {
		if (selected[e]) {
			e_set_.push_back(e);
			delta[e] = min(delta[e], 12); // real neuron firing window
			sampling_delta_.push_back(delta[e]);
		}
	}
	direct_neuron_ = new deque<int>[e_set_.size()];
	for (i = 0; i < e_set_.size(); i++) {
		for (n = 0; n < NumOfNeurons; n++) {
			if (directConnect(n, e_set_[i], sampling_delta_[i]))
				direct_neuron_[i].push_back(n);
		}
	}
	en = omp_get_wtime();
	printf("used %fs.\n", en - st);
}

void LrsHandler::printConnectGraph() {
	printf("Connect graph:\n");
	int n, e, d, i;
	for (n = 0; n < NumOfNeurons; n++) {
		for (e = 0; e < NumOfElectrodes; e++) {
			i = 0;
			for (d = 0; d < MaxDelta; d++) {
				if (connect_graph_[n][e][d] > 0.05) {
					if (i == 0)
						printf("n=%3u, e=%3u | ", n, e);
					else if (i % 8 == 0) {
						printf("\n");
						printf("             | ");
					}
					printf("%2u:%.2f, ", d, connect_graph_[n][e][d]);
					i++;
				}
				else
					break;
			}
			if (i > 0)
				printf("\n");
		}
	}
}
void LrsHandler::printLrsMap() {
	printf("LrsMap:\n");
	float bw = 0;
	for (int i = 0; i < e_set_.size(); i++) {
		int e = e_set_[i];
		int d = sampling_delta_[i];
		bw += 1.0 / (d + 1);
		printf("\t%3u e=%3u delta=%3u ", i, e, d);
		printf("direct neurons: ");
		for (int j = 0; j < direct_neuron_[i].size(); j++)
			printf("%3u, ", direct_neuron_[i][j]);
		printf("\n");
	}
	printf("\tLRS bw ratio: %f\n", bw / NumOfElectrodes);
}

//========= get mnc and update delta
void LrsHandler::initlrs(char* delta) {
	int e;
	int ptr = 0;
	for (e = 0;e < NumOfElectrodes;e++) {
		if (e == e_set_[ptr]) {
			delta[e] = char(sampling_delta_[ptr]);
			ptr++;
		}
	}

}
