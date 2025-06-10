#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;


  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading param
  lambda_ = 3 - n_aug_;

  // Initial state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units & rads
  x_ = VectorXd(n_x_);
  x_.fill(0.0);


  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);Add commentMore actions

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_(i) = 1 / (2 * lambda_ + 2 * n_aug_);
  }


  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2.0;

  // time when the state is true, in us
  time_us_ = 0.0;


  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // Check if the UKF has been initialized with the first measurement
  if (!is_initialized_) {
    // Initialize the state vector based on the first measurement
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // LIDAR provides direct x,y position measurements
      // Set the state with the initial location and zero velocity
      // State vector: [px, py, v, yaw, yawd]
      x_ << meas_package.raw_measurements_[0],  // px - x position
            meas_package.raw_measurements_[1],  // py - y position
            0,                                  // v - velocity (unknown, set to 0)
            0,                                  // yaw - yaw angle (unknown, set to 0)
            0;                                  // yawd - yaw rate (unknown, set to 0)
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // RADAR provides polar coordinates: range, bearing, range rate
      double rho = meas_package.raw_measurements_[0];      // range (distance)
      double phi = meas_package.raw_measurements_[1];      // bearing (angle)
      double rho_dot = meas_package.raw_measurements_[2];  // range rate (radial velocity)

      // Convert polar coordinates to cartesian coordinates
      // Set the state with the initial location and zero velocity
      x_ << rho * cos(phi),  // px - convert range/bearing to x position
            rho * sin(phi),  // py - convert range/bearing to y position
            0,               // v - velocity (unknown, set to 0)
            0,               // yaw - yaw angle (unknown, set to 0)
            0;               // yawd - yaw rate (unknown, set to 0)
    }
    else {
      // Invalid sensor type - print error and exit
      std::cerr << "UKF::ProcessMeasurement() error: cannot initialize because of invalid measurement sensor type " << meas_package.sensor_type_<< std::endl;
      exit(EXIT_FAILURE);
    }

    // Store the timestamp and mark the filter as initialized
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    // Debug output showing initialization
    if (bDebug)
      std::cout << "UKF::ProcessMeasurement() initializes for sensor type " << meas_package.sensor_type_ << " - state vector x:\n" << x_ << std::endl;

    return;  // Exit early after initialization
  }

  // Calculate time elapsed since last measurement
  // Convert from microseconds to seconds
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;  // Update stored timestamp

  // PREDICTION STEP: Predict the state and covariance to the current time
  Prediction(dt);
  if (bDebug)
    std::cout << "UKF::ProcessMeasurement() predicts the state vector x:\n" << x_ << std::endl;

  // UPDATE STEP: Update the predicted state with the new measurement
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    // Use LIDAR-specific update function (linear measurement model)
    UpdateLidar(meas_package);
    if (bDebug)
      std::cout << "UKF::ProcessMeasurement() Lidar update - state vector x:\n" << x_ << std::endl;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Use RADAR-specific update function (non-linear measurement model)
    UpdateRadar(meas_package);
    if (bDebug)
      std::cout << "UKF::ProcessMeasurement() Radar update - state vector x:\n" << x_ << std::endl;
  }
  else {
    // Invalid sensor type during update - print error and exit
    std::cerr << "UKF::ProcessMeasurement() error: cannot update measurement because of invalid measurement sensor type " << meas_package.sensor_type_ << std::endl;
    exit(EXIT_FAILURE);
  }

  return;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}