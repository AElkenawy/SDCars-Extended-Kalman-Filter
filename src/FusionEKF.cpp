#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;


/** Constructor **/

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;
  
  /* ############ EKF Algorithm Matrices Instantiations ############ */
  
  // Laser measurement covariance matrix [R_laser]
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Radar measurement covariance matrix [R_radar]
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  // Laser measurement function matrix [H_laser] 
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // Radar Measurement function Jacobian matrix [Hj] 
  Hj_ = MatrixXd(3, 4);
   
  // State covariance matrix [P]
  ekf_.P_ = MatrixXd(4, 4);

  // State transition matrix [F]
  ekf_.F_ = MatrixXd(4, 4);
  
  // Process covariance matrix [Q]
  ekf_.Q_ = MatrixXd(4, 4);
}

/** Destructor **/

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
  /* ############ EKF Algorithm Matrices Intialization ############ */
  if (!is_initialized_) {

    // Timestamping
    previous_timestamp_ = measurement_pack.timestamp_;

    // State vector intialization (first measurement)
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;
    
    // State initialization in case of a Radar measurement received
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar measurement polar to cartesian coordinates conversion
      float px0 =  measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      float py0 =  measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      
      ekf_.x_(0) = px0;
      ekf_.x_(1) = py0;
   }
    
    // State initialization in case of a Laser measurement received
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      float px0 =  measurement_pack.raw_measurements_[0];
      float py0 =  measurement_pack.raw_measurements_[1];
      
      ekf_.x_(0) = px0;
      ekf_.x_(1) = py0;
   }
    
  // State covariance matrix [P] intialization
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 200, 0,
             0, 0, 0, 200;
    
  // State transition matrix [F] intialization
  ekf_.F_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    
    is_initialized_ = true;
    return;
  }

/* ############ Prediction Step ############ */
  
  // elapsed time dt caluclation (in seconds)
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // State transition matrix [F] update  (due to elapsed time)
  ekf_.F_ (0,2) = dt;
  ekf_.F_ (1,3) = dt;
  
  // Process noise covariance matrix [Q] update
  
  // High dt exponents
  float dt_2 = pow(dt,2);
  float dt_3 = pow(dt,3);
  float dt_4 = pow(dt,4);
  
  // Process noise variances
  float noise_ax = 9;
  float noise_ay = 9;

  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0,  dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0,  dt_3/2*noise_ay, 0, dt_2*noise_ay;

  // Performing prediction step
  ekf_.Predict();

/* ############ Update Step ############ */
  
  // Radar update
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Constructing Radar Jacobian matrix [Hj] 
    Hj_ = tools.CalculateJacobian(ekf_.x_);

    // Storing [Hj] and [R_radar] in ekf instance
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    
    // Performing Radar EKF update step
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  
  // Laser update
  else {
    // Storing [H_laser] and [R_laser] in ekf instance
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_; 
    
    // Performing Laser KF update step
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Print updated states 
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
