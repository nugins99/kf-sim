#pragma once
#include <boost/property_tree/ptree.hpp>
#include <memory>
#include <string>

#include "CircleVehicle.h"
#include "StationaryVehicle.h"
#include "VehicleTruthModel.h"

/**
 * @brief Factory for vehicle truth models.
 * @param scenario Name of scenario ("stationary" or "circle").
 * @return Unique pointer to VehicleTruthModel.
 */
std::unique_ptr<VehicleTruthModel> createVehicleTruthModel(const std::string& scenario);

/**
 * @brief Factory for vehicle truth models from ptree config.
 * @param pt boost::property_tree::ptree object.
 * @return Unique pointer to VehicleTruthModel.
 */
std::unique_ptr<VehicleTruthModel> createVehicleTruthModelFromConfig(
    const boost::property_tree::ptree& pt);
