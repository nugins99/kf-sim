#include <Eigen/Dense>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iomanip>
#include <ios>
#include <iostream>

#include "IMUSensor6D.h"
#include "KalmanFilter.h"
#include "KalmanFilterFactory.h"
#include "OutputFactory.h"
#include "PositionDepthSensor.h"
#include "SensorPreprocessor.h"
#include "SensorPreprocessorInterface.h"
#include "Types.h"  // Include the Types header for DOF, StateVec, StateMat
#include "VehicleTruthModelFactory.h"

boost::property_tree::ptree readTreeFromFile(const std::string& filename)
{
    boost::property_tree::ptree pt;
    try
    {
        boost::property_tree::ini_parser::read_ini(filename, pt);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error reading config: " << e.what() << std::endl;
        throw;
    }
    return pt;
}

std::unique_ptr<SensorPreprocessorInterface> createSensorPreprocessorFromConfig(
    const boost::property_tree::ptree& pt)
{
    // For now, always return SensorPreprocessor. Extend here for other implementations.
    return std::make_unique<SensorPreprocessor>(pt);
}

int main(int argc, char* argv[])
{
    std::string configPath = "../../config/truth_model.cfg";
    if (argc > 1)
    {
        configPath = argv[1];
    }
    auto pt = readTreeFromFile(configPath);

    const double dt = pt.get<double>("sim.dt", 0.1);    // fallback to 0.1
    const int steps = pt.get<int>("sim.steps", 60000);  // fallback to 60000

    // Create truth model from ptree
    auto truth = createVehicleTruthModelFromConfig(pt);

    // Initial state: match truth
    auto initialState = truth->getState();  // Use the truth model's getState method
    StateMat initialCovariance = StateMat::Identity() * 1e-2;

    // Process and measurement noise
    StateMat Q = StateMat::Identity() * 1e-4;
    StateMat R = StateMat::Identity() * 1e-2;
    auto kf = createKalmanFilterFromConfig<DOF>(pt, dt, Q, R);
    kf->init(initialState, initialCovariance);

    // Create sensors from config
    auto preprocessor = createSensorPreprocessorFromConfig(pt);
    preprocessor->initialize(truth->getVelocity());  // Initialize with truth velocity

    auto output = createOutputFromConfig(pt, steps);
    for (int i = 0; i < steps; ++i)
    {
        auto truthState = truth->next(dt);
        kf->predict();
        auto measurement = preprocessor->getMeasurement(truth.get(), dt, i, kf->getState());
        if (!measurement.hasNaN())
        {
            // Update the Kalman filter with the measurement
            // If the measurement is invalid (e.g., NaN), skip the update
            // This is a simple check; you might want to implement a more robust check
            // based on your specific requirements.
            kf->update(measurement);
        }
        output->report(i, truthState, kf->getState(), kf->getCovariance());
    }
    return 0;
}