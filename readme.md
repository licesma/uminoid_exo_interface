# uminoid_exo_interface
Manages values produced from an exoskeleton. Python acts as orchestrator; the C++ code under `/cpp` executes the values read from the exoskeleton.

## Prerequisites

```bash
# Build tools
sudo apt-get install git wget cmake build-essential

# yaml-cpp
sudo apt-get install libyaml-cpp-dev

# Librealsense dependencies
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

# Librealsense Linux backend
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```

## Usage

### Python environment

```bash
conda create -n uminoid_exo python=3.11 && conda run -n uminoid_exo pip install -r requirements.txt
```

## Handle C++
```bash
cd cpp
```


### Configure CMAKE
(Only necessary when CMakeLists.txt is changed)
```bash
cmake -B build && cmake --build build
```


### Build
**Build all**
```bash
cmake --build build
```

**Build a specific file**
Build a specific File
```bash
cmake --build build --target <target_file>
```


### Execute
```bash
./build/<target_file>
```
