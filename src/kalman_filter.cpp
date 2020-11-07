#include "kalman_filter.h"

#define PI 3.14159265


using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /* ##### KF State prediciton equations ##### */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /* ##### KF update equations ##### */
  
  // Predicted measurement vector [z_pred]
  VectorXd z_pred = H_ * x_;
  // Innovation residual vector [y]
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  // Innovation covariance matrix [S]
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  // New estimation
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /* ##### EKF update equations ##### */
  
  // Constructing non-linear measurement function [h(x')]
  
  // Cartesian position coordinates and velocities
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  // Mapping cartesian into polar coordinates
  float rho = sqrt (px * px + py * py);
  float phi = atan2(py,px);
  // rho_dot zero devision avoidance
  if (rho < 0.0001){
    rho = 0.0001;}
  float rho_dot = (px * vx + py * vy)/rho;

  // Measurement function [h(x')]
  VectorXd h =  VectorXd(3);
  h << rho, phi, rho_dot;
  VectorXd y = z - h;
  
  // Phi angle normalization to suit EKF algorithm
  // using range of -PI < phi < PI
  if (y(1) < -PI){
  y(1) += 2*PI;}
  else if (y(1) > PI){   
  y(1) -= 2*PI;}
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //New estimation
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
