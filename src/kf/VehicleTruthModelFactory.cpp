#include "VehicleTruthModelFactory.h"

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
