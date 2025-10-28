#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"
#include "optimization_config.h"

class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  // Fixed-size measurement storage to avoid dynamic allocations
  OptimizedTypes::LidarMeasurementVector lidar_measurements_;
  OptimizedTypes::RadarMeasurementVector radar_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
