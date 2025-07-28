# RLController - RL Policy FSM Controller for H1 Robot

This FSM controller integrates reinforcement learning policies with mc\_rtc for controlling the H1 humanoid robot. It supports impedance control to translate RL policy joint position commands into torques.

## Requirements

This project requires [ONNX Runtime](https://onnxruntime.ai/) to run reinforcement learning models in the controller.

We recommend using the official prebuilt binary provided by Microsoft for simplicity and consistency.

### Using official ONNX Runtime release

#### Download the prebuilt release

Go to the official release page and download the appropriate archive for your platform: [https://github.com/microsoft/onnxruntime/releases](https://github.com/microsoft/onnxruntime/releases)

#### Install ONNX Runtime locally

Download (replace with latest version if needed) :
```sh
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
```

Extract to a permanent location :
```sh
mkdir -p ~/opt/
tar -xzf onnxruntime-linux-x64-1.22.0.tgz -C ~/opt/
mv ~/opt/onnxruntime-linux-x64-1.22.0 ~/opt/onnxruntime
```

Your directory structure will now look like:

```
~/opt/onnxruntime/
├── include/
│   └── onnxruntime_cxx_api.h
└── lib/
    └── libonnxruntime.so (or .a)
```

#### Build your controller with ONNX support

In your project :

```sh
mkdir -p build && cd build
```

Tell CMake where ONNX Runtime is installed :
```sh
cmake .. -DONNXRUNTIME_ROOT=~/opt/onnxruntime
```
Build and install :
```sh
make
make install
```

#### Optional: Use an environment variable

You can also define `ONNXRUNTIME_ROOT` as an environment variable instead of passing it to CMake, either in your terminal or, more durably, in your .bashrc or equivalent :

```bash
export ONNXRUNTIME_ROOT=~/opt/onnxruntime
```

//TODO finish README.md