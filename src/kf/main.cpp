#include <Eigen/Dense>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iomanip>
#include <ios>
#include <iostream>

#include "FileOutput.h"
#include "IMUSensor6D.h"
#include "KalmanFilter.h"
#include "MultiOutput.h"
#include "PositionDepthSensor.h"
#include "ProgressReporter.h"
#include "SensorPreprocessor.h"
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

    double dt = pt.get<double>("sim.dt", 0.1);    // fallback to 0.1
    int steps = pt.get<int>("sim.steps", 60000);  // fallback to 60000

    constexpr int DOF = 6;  // [x, y, z, vx, vy, vz]
    using StateVec = Eigen::Matrix<double, DOF, 1>;
    using StateMat = Eigen::Matrix<double, DOF, DOF>;

    // Create truth model from ptree
    std::unique_ptr<VehicleTruthModel> truth = createVehicleTruthModelFromConfig(pt);

    // Initial state: match truth
    Eigen::Vector3d true_pos = truth->getPosition();
    Eigen::Vector3d true_vel = truth->getVelocity();
    StateVec initial_state;
    initial_state << true_pos(0), true_pos(1), true_pos(2), true_vel(0), true_vel(1), true_vel(2);
    StateMat initial_cov = StateMat::Identity() * 1e-2;

    // Process and measurement noise
    StateMat Q = StateMat::Identity() * 1e-4;
    StateMat R = StateMat::Identity() * 1e-2;
    KalmanFilter<DOF> kf(dt, Q, R);
    kf.init(initial_state, initial_cov);

    // Create sensors from config
    SensorPreprocessor preprocessor(pt);
    preprocessor.initialize(true_vel);

    auto file_output = std::make_shared<FileOutput>("output.txt");
    auto progress_output = std::make_shared<ProgressReporter>(1000);
    MultiOutput output;
    output.addOutput(file_output);
    output.addOutput(progress_output);
    for (int i = 0; i < steps; ++i)
    {
        auto [tpos, tvel] = truth->next(dt);
        double tspeed = tvel.head<2>().norm();
        double theading = std::atan2(tvel(1), tvel(0)) * 180.0 / M_PI;
        auto kf_est = kf.getState();
        auto measurement = preprocessor.getMeasurement(truth.get(), dt, i, kf_est);
        kf.predict();
        kf.update(measurement);
        preprocessor.updateIntegratedVel(kf.getState());
        StateVec est = kf.getState();
        double espeed = est.segment<2>(3).norm();
        double eheading = std::atan2(est(4), est(3)) * 180.0 / M_PI;
        double position_error = (tpos - est.head<3>()).norm();
        output.report(i, tpos, tvel, tspeed, theading, est, espeed, eheading, measurement.head<3>(),
                      position_error);
    }
    return 0;
}