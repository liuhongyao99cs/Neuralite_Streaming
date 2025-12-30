# Neuralite: Enabling Wireless High-Resolution Brain-Computer Interfaces

Welcome to the streaming control of **Neuralite**, which is a server-driven framework to enable wireless iBCIs.

## Workflow
The high-level workflow is as follows:

![Neuralite Workflow Diagram](https://github.com/liuhongyao99cs/Neuralite/blob/main/images/neuralite_workflow.png)

Neuralite first operates an offline and training stage. The server gathers full-resolution iBCI neural signal frames from the headstage (Note: Constrained by the throughput, the frame is not continuous in time, but it can still capture enough spikes for training). Then, the server constructs a minimal stream, which uses least throughput but can detect activities from all neurons. The headstage first sends this stream to the server. Ther server identifies probobly firing neurons and instructs the headstage to send high-resolution signal region of the firing neurons. With spike sorting, spikes are extracted and further used in downstream decoders.





If server runs a trace experiment:
in server.cpp, comment out step 1-3 in main function. In function computeHdsReq, comment out lines 650-714.

Entire project is shared in Google drive:
https://drive.google.com/drive/folders/149wrkDl0VkA4vgzA4Y0_X2-GQYss2SaN?usp=drive_link
