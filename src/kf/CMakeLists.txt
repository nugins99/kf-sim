add_library(kf-sim-lib
    IMUSensor6D.cpp
    PositionDepthSensor.cpp
    StationaryVehicle.cpp
    CircleVehicle.cpp
    VehicleTruthModelFactory.cpp
    SensorPreprocessor.cpp)

target_link_libraries(kf-sim-lib PUBLIC
    Eigen3::Eigen
    Boost::program_options
    Boost::boost)

target_include_directories(kf-sim-lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)

add_executable(kf-sim
    main.cpp)

target_link_libraries(kf-sim PRIVATE
    kf-sim-lib)