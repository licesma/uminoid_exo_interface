# uminoid_exo_interface
Manages values produced from an exoskeleton. Python acts as orchestrator; the C++ code under `/cpp` executes the values read from the exoskeleton.

## Usage

### Build C++

```bash
cd cpp
cmake -B build && cmake --build build
```
