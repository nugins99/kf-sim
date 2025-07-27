# KF-Sim

KF-Sim is a simulation project for Kalman Filter-based vehicle tracking and
estimation. It provides a modular framework for simulating vehicles, sensors,
and estimation algorithms, primarily using C++ and CMake.

**Note:** Most of the work here in this project was done with AI assistance. I have
basic idea of what a Kalman filter does and how it works.  This project is just
something I did to explore creating an application. 

## Requirements
- CMake (>= 3.15)
- Conan (for dependency management)
- C++17 compatible compiler (e.g., GCC, Clang, or Apple Clang)
- Boost
- Eigen3
- GTest (for unit testing)

## Build Instructions
1. **Install dependencies with Conan:**
   ```sh
   conan install . --output-folder=build --build=missing
   ```
2. **Configure the project with CMake:**
   ```sh
   cmake -S . -B build
   ```
3. **Build the project:**
   ```sh
   cmake --build build
   ```

## Run Instructions
- Executables are located in `build/bin/` after building.
- To run the main simulation:
  ```sh
  ./build/bin/kf-sim
  ```

## Viewing Outputs 
Running the simulation generates an "output.txt".

```
    cd build/bin
    ../../scripts/plot
```
**Example**
![alt text](data/plot.svg)

## Testing
- To run unit tests:
  ```sh
  ctest --test-dir build
  ```

## Notes
- For custom build presets, see `CMakePresets.json`.
- For more details, refer to the source code in `src/` folder.
