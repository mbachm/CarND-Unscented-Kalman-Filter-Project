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

  //set radar measurement dimension
  n_z_radar_ = 3;
  
  //set laser measurement dimension
  n_z_laser_ = 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_ << 0, 0, 0, 0, 0; //px, py, v, yaw, yawdot

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ = MatrixXd::Identity(n_x_, n_x_); //Like shown in UKF lesson 32

  // initial predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //time when the state is true, in us
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;

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
  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2 * n_aug_ + 1; i++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  H_laser_ = MatrixXd(2, n_x_);
    
  //measurement covariance matrix for laser - taken from EKF project
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement matrix - taken from EKF project
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;
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
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    //Time is measured in seconds.
    Prediction(dt);
    UpdateRadar(meas_package);
    time_us_ = meas_package.timestamp_;
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    //Time is measured in seconds.
    Prediction(dt);
    UpdateLidar(meas_package);
    time_us_ = meas_package.timestamp_;
  }
}

void UKF::Initialize(MeasurementPackage meas_package) {
  // Initialize the state ekf_.x_ with the first measurement.
  cout << "UKF: " << endl;
  
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    /**
     * Convert radar from polar to cartesian coordinates and initialize state.
     */
    double rho = meas_package.raw_measurements_[0];
    double phi = normalize_angle(meas_package.raw_measurements_[1]);
    double rhodot = meas_package.raw_measurements_(2);
    double px = rho * cos(phi);
    double py = rho * sin(phi);
    double vx = rhodot * cos(0);
    double vy = rhodot * sin(0);
    
    //initialize state
    x_[0] = px;
    x_[1] = py;
    x_[3] = sqrt(vx * vx + vy * vy);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
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
    
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&Xsig_aug);
  SigmaPointPrediction(&Xsig_aug, delta_t);
  PredictMeanAndCovariance();
    
  std::cout << "Predicted state" << std::endl;
  std::cout << x_ << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P_ << std::endl;
  std::cout << std::endl << std::endl;
  return;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_laser_);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_laser_, n_z_laser_);
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_laser_, 2 * n_aug_ + 1);
  
  PredictLaserMeasurement(&z_pred, &S, &Zsig);
  
  VectorXd z(n_z_laser_);
  double px = meas_package.raw_measurements_[0];
  double py = meas_package.raw_measurements_[1];
  z << px, py;
  
  LaserUpdateState(z_pred, S, z, Zsig);
  return;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
    
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
    
  PredictRadarMeasurement(&z_pred, &S, &Zsig);

  VectorXd z(n_z_radar_);
  double rho = meas_package.raw_measurements_[0];
  double phi = normalize_angle(meas_package.raw_measurements_[1]);
  double rhodot = meas_package.raw_measurements_[2];
  z << rho, phi, rhodot;
    
  RadarUpdateState(z_pred, S, z, Zsig);
  return;
}

double UKF::normalize_angle(double angle) {
  /**
   * Normalizes angle wirth atan2
   */
  return atan2(sin(angle), cos(angle));
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_aug) {
  /**
   * Creates the augmented sigma point matrix as shown in the lessons.
   */
    
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  
  //create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
    
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;
    
  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  Xsig_aug->col(0) = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug->col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug->col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  return;
}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, double delta_t) {
  /**
   * Uses the augmented sigma points matrix and delta_t to predict the
   * values for Xsig_pred_ and stores them as shown in the lessons.
   */
  //predict sigma points
  for (int i = 0; i< 2 * n_aug_ + 1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_out->coeffRef(0,i);
    double p_y = Xsig_out->coeffRef(1,i);
    double v = Xsig_out->coeffRef(2,i);
    double yaw = Xsig_out->coeffRef(3,i);
    double yawd = Xsig_out->coeffRef(4,i);
    double nu_a = Xsig_out->coeffRef(5,i);
    double nu_yawdd = Xsig_out->coeffRef(6,i);
    
    //predicted state values
    double px_p, py_p;
    
    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * (sin (yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }
    
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;
    
    //add noise
    double c0 = 0.5 * nu_a * delta_t *delta_t;
    px_p = px_p + c0 * cos(yaw);
    py_p = py_p + c0 * sin(yaw);
    v_p = v_p + nu_a*delta_t;
    
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;
    
    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  return;
}

void UKF::PredictMeanAndCovariance() {
  /*
   * Predicts the state mean and state covariance matrix as shown in the lessons.
   */
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  
  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3) = normalize_angle(x_diff(3));
    
    P_= P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
  return;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd *Zsig_out) {
  /*
   * Predict radar measurement as shown in lesson
   */
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
    
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = normalize_angle(Xsig_pred_(3,i));
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;
    // measurement model
    //r
    Zsig(0,i) = sqrt(p_x * p_x + p_y * p_y);
    if(Zsig(0,i) < 0.001){
      Zsig(0,i) = 0.001;
    }
    //phi
    if (p_x == 0) {
      cout << "Something strange happend" << endl;
    }
    Zsig(1,i) = normalize_angle(atan2(p_y, p_x));
    //r_dot
    if (p_x == 0 && p_y == 0) {
      cout << "Something very strange happend" << endl;
    }
    Zsig(2,i) = (p_x * v1 + p_y * v2) / Zsig(0,i);
  }
    
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
    
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    //angle normalization
    z_diff(1) = normalize_angle(z_diff(1));
    
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
    
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_radar_,n_z_radar_);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0,std_radrd_*std_radrd_;
  S = S + R;
    
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
  return;
}

void UKF::RadarUpdateState(VectorXd z_pred, MatrixXd S, VectorXd  z, MatrixXd Zsig) {
  /*
   * Update state with radar measurements and previously predicted values
   */

  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
    
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    z_diff(1) = normalize_angle(z_diff(1));
        
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3) = normalize_angle(x_diff(3));
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
    
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
    
  //residual
  VectorXd z_diff = z - z_pred;
    
  //angle normalization
  z_diff(1) = normalize_angle(z_diff(1));
    
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
    
  //Calculate NIS
  double NIS = z_diff.transpose() * S.inverse() * z_diff;
    
  std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
  std::cout << "NIS radar: " << std::endl << NIS << std::endl << std::endl;

  return;
}

void UKF::PredictLaserMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd *Zsig_out) {
  /*
   * Predict laser measurement
   */
  MatrixXd Zsig = MatrixXd(n_z_laser_, 2 * n_aug_ + 1);
  
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    
    // measurement model
    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_laser_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_laser_, n_z_laser_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_laser_,n_z_laser_);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  S = S + R;
  
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
  return;
}

void UKF::LaserUpdateState(VectorXd z_pred, MatrixXd S, VectorXd z, MatrixXd Zsig) {
  /*
   * Update state with radar measurements and previously predicted values
   */
  MatrixXd Tc = MatrixXd(n_x_, n_z_laser_);
  
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //residual
  VectorXd z_diff = z - z_pred;
  
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  //Calculate NIS
  double NIS = z_diff.transpose() * S.inverse() * z_diff;
  
  std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
  std::cout << "NIS laser: " << std::endl << NIS << std::endl << std::endl;
  
  return;
}
