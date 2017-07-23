#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Radar measurement dimension
  int n_z_;
  
  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* measurement covariance matrix, taken from EKF project
  Eigen::MatrixXd R_laser_;
    
  ///* measurement matrix, taken from EKF project
  Eigen::MatrixXd H_laser_;
  
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);
    
  /**
   * Initialize UKF with first measurement
   * @param meas_package The first measurement
   */
  void Initialize(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Normalizes phi with atan2
   * @param angle The angle of a measurement to normalize
   */
  double normalize_angle(double angle);
    
  /**
   * Generates sigma points with augmentation with process noise for the prediction step
   * @param Xsig_aug Matrix to store the sigma points
   */
  void AugmentedSigmaPoints(MatrixXd* Xsig_aug);
  
  /*
   * Predict sigma points
   * @param Xsig_out Matrix to store the predicted sigma points
   * @param delta_t Time difference in seconds between the last measurement and the new one
   */
  void SigmaPointPrediction(MatrixXd* Xsig_out, double delta_t);
    
  /*
   * Predict mean and covariance matrix in prediction step
   */
  void PredictMeanAndCovariance();

  /*
   * Predict sigma points
   * @param z_out Vector to store mean predicted measurement
   * @param S_out Matrix to store measurement covariance matrix
   * @param Zsig_out Matrix to store sigma points in measurement space
   */
  void PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd *Zsig_out);

  /*
   * Predict sigma points
   * @param z_pred Mean predicted measurement vector
   * @param S Predicted measurement covariance matrix
   * @param z Radar measurement vector
   * @param Zsig Matrix of sigma points in measurement space
   */
  void RadarUpdateState(VectorXd z_pred, MatrixXd S, VectorXd z, MatrixXd Zsig);

};

#endif /* UKF_H */
