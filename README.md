# KF-Sim

KF-Sim is a simulation project for Kalman Filter-based vehicle tracking and estimation. It provides a modular framework for simulating vehicles, sensors, and estimation algorithms, primarily using C++ and CMake.

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
  ./build/bin/<executable_name>
  ```
  Replace `<executable_name>` with the actual binary name (e.g., `kf_sim`).

## Testing
- To run unit tests:
  ```sh
  ctest --test-dir build
  ```

## Notes
- For custom build presets, see `CMakePresets.json`.
- For more details, refer to the source code in `src/` and `build/` folders.
