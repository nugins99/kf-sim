#pragma once
#include <memory>
#include <string>
#include "VehicleTruthModel.h"
#include "StationaryVehicle.h"
#include "CircleVehicle.h"

/**
 * @brief Factory for vehicle truth models.
 * @param scenario Name of scenario ("stationary" or "circle").
 * @return Unique pointer to VehicleTruthModel.
 */
std::unique_ptr<VehicleTruthModel> createVehicleTruthModel(const std::string& scenario);
