#include "VehicleTruthModelFactory.h"

#include <boost/property_tree/ptree.hpp>
#include <iostream>

std::unique_ptr<VehicleTruthModel> createVehicleTruthModel(const std::string& scenario)
{
    if (scenario == "stationary")
    {
        return std::make_unique<StationaryVehicle>();
    }
    else if (scenario == "circle")
    {
        constexpr double nm_to_ft = 6076.12;
        constexpr double knots_to_ftps = 1.68781;
        double radius = nm_to_ft;            // 1 nm
        double speed = 5.0 * knots_to_ftps;  // 5 knots
        double depth = -20.0;                // Example depth in feet
        return std::make_unique<CircleVehicle>(radius, speed, depth);
    }
    return nullptr;
}

std::unique_ptr<VehicleTruthModel> createVehicleTruthModelFromConfig(
    const boost::property_tree::ptree& pt)
{
    std::string scenario = pt.get<std::string>("truth_model.scenario", "stationary");
    if (scenario == "stationary")
    {
        double pos_x = pt.get<double>("stationary.pos_x", 0.0);
        double pos_y = pt.get<double>("stationary.pos_y", 0.0);
        double pos_z = pt.get<double>("stationary.pos_z", 0.0);
        double orient_x = pt.get<double>("stationary.orient_x", 0.0);
        double orient_y = pt.get<double>("stationary.orient_y", 0.0);
        double orient_z = pt.get<double>("stationary.orient_z", 0.0);
        return std::make_unique<StationaryVehicle>(Eigen::Vector3d(pos_x, pos_y, pos_z),
                                                   Eigen::Vector3d(orient_x, orient_y, orient_z));
    }
    else if (scenario == "circle")
    {
        double radius = pt.get<double>("circle.radius", 6076.12);
        double speed = pt.get<double>("circle.speed", 8.43905);
        double depth = pt.get<double>("circle.depth", -20.0);
        double center_x = pt.get<double>("circle.center_x", 0.0);
        double center_y = pt.get<double>("circle.center_y", 0.0);
        double center_z = pt.get<double>("circle.center_z", 0.0);
        return std::make_unique<CircleVehicle>(radius, speed, depth,
                                               Eigen::Vector3d(center_x, center_y, center_z));
    }
    return nullptr;
}
