# Neuralite: Enabling Wireless High-Resolution Brain-Computer Interfaces

Welcome to the streaming control of **Neuralite**, which is a server-driven framework to enable wireless iBCIs.

## Workflow Overview
The high-level workflow is as follows:

![Neuralite Workflow Diagram](https://github.com/liuhongyao99cs/Neuralite/blob/main/images/neuralite_workflow.png)

Neuralite operates in two primary stages: **Offline Training** and **Online Streaming**.

### 1. Offline & Training Stage
During this initial phase, the server aggregates full-resolution iBCI neural signal frames from the headstage.

> **Note:** Due to wireless throughput constraints, the frames gathered in this stage are not continuous in time; however, they capture sufficient spike density to train the system effectively.

### 2. Online Streaming Stage
Once initialized, the system enters a bandwidth-efficient feedback loop:

1.  **Stream Construction:** The server constructs a **minimal stream** protocol that minimizes throughput usage while retaining the ability to detect activity from all neurons.
2.  **Transmission:** The headstage transmits this minimal stream to the server.
3.  **Targeted Querying:** The server analyzes the stream to identify potentially firing neurons. It then instructs the headstage to transmit **high-resolution signal regions** specifically for those active neurons.
4.  **Processing:** The server receives the requested high-resolution data, performs spike sorting, and forwards the extracted spikes to downstream decoders.

## Trace-driven experiments

To EASILY benchmark the efficiency and streaming quality, we provide the trace-driven experiments of Neuralite. 

#### Prequisit
1. **Spike sorting kernel**: For spike sorting, please refer to Kilosort 3 (https://github.com/neurodisney/Kilosort3), which is the SOTA template-matching spike sorting algorithm.
2. **Downstream neural decoders**: neural decoders translate spikes into intentions of subject animals, such as CEBRA
3. **VScode and ESP-IDF extention**
4. **Visual studio community 2022**

#### Code structure
1. **Firmware in ESP32**:


If server runs a trace experiment:
in server.cpp, comment out step 1-3 in main function. In function computeHdsReq, comment out lines 650-714.

Entire project is shared in Google drive:
https://drive.google.com/drive/folders/149wrkDl0VkA4vgzA4Y0_X2-GQYss2SaN?usp=drive_link
