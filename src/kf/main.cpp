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

int main(int argc, char* argv[])
{
    std::string config_path = "../../config/truth_model.cfg";
    if (argc > 1)
    {
        config_path = argv[1];
    }
    auto pt = readTreeFromFile(config_path);

    const double dt = pt.get<double>("sim.dt", 0.1);    // fallback to 0.1
    const int steps = pt.get<int>("sim.steps", 60000);  // fallback to 60000

    // Create truth model from ptree
    std::unique_ptr<VehicleTruthModel> truth = createVehicleTruthModelFromConfig(pt);

    // Initial state: match truth
    auto initialState = truth->getState();  // Use the truth model's getState method
    StateMat initialCovariance = StateMat::Identity() * 1e-2;

    // Process and measurement noise
    StateMat Q = StateMat::Identity() * 1e-4;
    StateMat R = StateMat::Identity() * 1e-2;
    auto kf = createKalmanFilterFromConfig<DOF>(pt, dt, Q, R);
    kf->init(initialState, initialCovariance);

    // Create sensors from config
    SensorPreprocessor preprocessor(pt);
    preprocessor.initialize(truth->getVelocity());  // Initialize with truth velocity

    auto output = createOutputFromConfig(pt, steps);
    for (int i = 0; i < steps; ++i)
    {
        auto truthState = truth->next(dt);
        auto measurement = preprocessor.getMeasurement(truth.get(), dt, i, kf->getState());
        kf->predict();
        kf->update(measurement);
        output->report(i, truthState, kf->getState(), kf->getCovariance());
    }
    return 0;
}