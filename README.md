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

1.  **Stream Construction:** The server constructs a **minimal stream** (Lrs) protocol that minimizes throughput usage while retaining the ability to detect activity from all neurons.
2.  **Transmission:** The headstage transmits this minimal stream to the server.
3.  **Targeted Querying:** The server analyzes the stream to identify potentially firing neurons. It then instructs the headstage to transmit **high-resolution signal regions** (Hds) specifically for those active neurons.
4.  **Processing:** The server receives the requested high-resolution data, performs spike sorting, and forwards the extracted spikes to downstream decoders.

## Trace-driven Experiments

To facilitate easy benchmarking of efficiency and streaming quality, we provide a suite of trace-driven experiments for Neuralite.

### 🛠️ Prerequisites

1.  **Spike Sorting Kernel**: We utilize [Kilosort 3](https://github.com/neurodisney/Kilosort3), the state-of-the-art template-matching spike sorting algorithm.
2.  **Downstream Neural Decoders**: Decoders used to translate spikes into subject intentions (e.g., [CEBRA](https://cebra.ai/)).
3.  **VS Code & ESP-IDF Extension**: Required to compile and flash the firmware for the ESP32 headstage.
4.  **Visual Studio Community 2022**: Required to run the server application on Windows 11.

### 📂 Trace Data & Input Files

All trace files are available for download [here](https://drive.google.com/drive/folders/149wrkDl0VkA4vgzA4Y0_X2-GQYss2SaN?usp=drive_link).

1.  **Neuron Information**
    * `templates.txt`: Contains spike template waveforms.
    * `power.txt`, `sigma.txt`: Contains characteristics extracted via Kilosort 3.
2.  **LrsMap.txt**
    * Records the **Minimal Stream's** electrode and downsampling configuration.
    * **Format:** A line `M N` indicates that electrode `M` is sampled once every `N` samples.
    * **Function:** This ensures the system detects the power of all neurons recorded by the electrode while minimizing throughput. It is trained based on the neuron information files.
3.  **Hdsreq.txt**
    * Represents the region information required for the **High-Resolution Stream**, identified by the server during runtime.
    * **Format:** `{start time stamp, electrode id, downsample factor}`.

### 🏗️ Code Structure

1.  **Headstage Firmware (`Headstage_git/`)**
    * Designed for the **ESP32**.
    * Utilizes the MCU's dual-core structure to handle the **Lrs** (Low-resolution stream) and **Hds** (High-resolution stream) simultaneously.
2.  **Server (`server.sln`)**
    * A Visual Studio solution that can be executed directly on Windows with Visual Studio 2022.

### 📊 Output

* **Data Storage:** Received data is stored in the `frame` class.
* **Logs:** The system records a frame drop sequence file to track transmission reliability.

## 📈 Experimental Results

<p align="center">
  <img src="https://github.com/liuhongyao99cs/Neuralite/blob/main/images/result.png" alt="Neuralite Experimental Results" width="80%">
</p>

Our experiments demonstrate that Neuralite significantly enhances wireless reliability without compromising data fidelity:

* **Robustness:** Reduces the frame drop rate by **15–24x**.
* **Accuracy:** Maintains a spike sorting error rate of **<3%**.

## 🖊️ Citation

If you find our work or this code useful for your research, please cite our papers:

### Neuralite (MobiCom '24)
```bibtex
@inproceedings{liu2024neuralite,
  title={Neuralite: Enabling Wireless High-Resolution Brain-Computer Interfaces},
  author={Liu, Hongyao and Wang, Junyi and Zhai, Liuqun and Fang, Yuguang and Huang, Jun},
  booktitle={Proceedings of the 30th Annual International Conference on Mobile Computing and Networking},
  pages={984--999},
  year={2024}
}
```
@inproceedings{liu2024neuron,
  title={Neuron-aware brain-to-computer communication for wireless intracortical bci},
  author={Liu, Hongyao and Wang, Junyi and Chen, Xi and Huang, Jun},
  booktitle={Proceedings of the 25th International Workshop on Mobile Computing Systems and Applications},
  pages={107--113},
  year={2024}
}
