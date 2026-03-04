# uminoid_exo_interface
Manages values produced from an exoskeleton. Python acts as orchestrator; the C++ code under `/cpp` executes the values read from the exoskeleton.

## Usage

### Python environment

```bash
conda create -n uminoid_exo python=3.11 && conda run -n uminoid_exo pip install -r requirements.txt
```

### Build C++

```bash
cd cpp
cmake -B build && cmake --build build
```
