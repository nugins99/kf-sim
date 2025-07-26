find_package(Eigen3 REQUIRED) find_package(Boost REQUIRED COMPONENTS program_options)

    add_executable(kf -
                   sim IMUSensor6D.cpp PositionDepthSensor.cpp StationaryVehicle.cpp CircleVehicle
                       .cpp VehicleTruthModelFactory.cpp SensorPreprocessor.cpp main.cpp)
        target_link_libraries(kf - sim PRIVATE Eigen3::Eigen Boost::program_options Boost::boost)