find_package(Eigen3 REQUIRED)

add_executable(kf-sim
    IMUSensor6D.cpp 
    PositionDepthSensor.cpp
    StationaryVehicle.cpp
    CircleVehicle.cpp
    VehicleTruthModelFactory.cpp
    SensorPreprocessor.cpp
    main.cpp)
target_link_libraries(kf-sim PRIVATE Eigen3::Eigen)