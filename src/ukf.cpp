#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // false as we did not initialized the filter
  is_initialized_ = false;
    
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  //set state dimension
  n_x_ = 5;
    
  // Augmented state dimension
  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_ << 0, 0, 0, 0, 0; //px, py, v, yaw, yawdot

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ = MatrixXd::Identity(n_x_, n_x_); //Like shown in UKF lesson 32

  // initial predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_x_ + 1);

  //time when the state is true, in us
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  // initial weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    Initialize(meas_package);
    return;
  }

  //Time is measured in seconds.
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  /**
  TODO: Calculate everything necessary before prediction step
   * GenerateSigmaPoints
   * AugmentedSigmaPoints
   *
  */
  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else {
    UpdateLidar(meas_package);
  }
}

void UKF::Initialize(MeasurementPackage meas_package) {
    // Initialize the state ekf_.x_ with the first measurement.
    cout << "UKF: " << endl;
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        /**
         Convert radar from polar to cartesian coordinates and initialize state.
         */
        double rho = meas_package.raw_measurements_[0];
        double phi = normalize_phi(meas_package.raw_measurements_[1]);
        double px = rho * cos(phi);
        double py = rho * sin(phi);
        
        //initialize state
        x_[0] = px;
        x_[1] = py;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        //initialize state.
        x_[0] = meas_package.raw_measurements_[0];
        x_[1] = meas_package.raw_measurements_[1];
    }
    time_us_ = meas_package.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:
   * GenerateSigmaPoints
   * AugmentedSigmaPoints?
   * SigmaPointPrediction
   * PredictMeanAndCovariance
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

double UKF::normalize_phi(double phi) {
    /**
     * Normalizes phi wirth atan2
     */
    return atan2(sin(phi), cos(phi));
}
