enable_testing()
find_package(GTest REQUIRED)

add_executable(UnitTest
    test_StationaryVehicle.cpp 
    test_IMUSensor6D.cpp
    test_CircleVehicle.cpp 
    test_SensorPreprocessor.cpp)
target_link_libraries(UnitTest PRIVATE 
    kf-sim-lib 
    gtest::gtest)

include(GoogleTest)
gtest_discover_tests(UnitTest)
