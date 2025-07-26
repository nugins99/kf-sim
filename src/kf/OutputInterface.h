#pragma once
#include <Eigen/Dense>
#include <string>

/**
 * @brief Abstract interface for simulation output reporting.
 */
class OutputInterface {
public:
    virtual ~OutputInterface() = default;
    /**
     * @brief Report a single simulation step.
     * @param step Simulation step index.
     * @param tpos True position.
     * @param tvel True velocity.
     * @param tspeed True speed.
     * @param theading True heading.
     * @param est Estimated state vector.
     * @param espeed Estimated speed.
     * @param eheading Estimated heading.
     * @param measurement Measurement vector.
     * @param position_error Position error.
     */
    virtual void report(int step,
                       const Eigen::Vector3d& tpos,
                       const Eigen::Vector3d& tvel,
                       double tspeed,
                       double theading,
                       const Eigen::Matrix<double, 6, 1>& est,
                       double espeed,
                       double eheading,
                       const Eigen::Vector3d& measurement,
                       double position_error) = 0;
};
