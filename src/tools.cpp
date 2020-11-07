#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
 /* ##### RMSE calculation ##### */
  
  // RMSE vector [rmse] instantiation 
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // Non-zero sized estimation vector assertation
  assert (estimations.size()!=0);
  // Equal-sized ground truth and estimation vectors
  assert (ground_truth.size()==estimations.size());

  // Squared residuals accumulatation 
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  // Mean calculation
  rmse = rmse/estimations.size();
  // Root-mean-square 
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  /* ##### Jacobian matrix [Hj] construction ##### */
  
  // Jacobian matrix [Hj] instantiation 
  MatrixXd Hj(3,4);
  Hj << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;
  
  // State parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Radar equation terms (cartesian)
  float t1 = px*px+py*py;
  float t2 = sqrt(t1);
  float t3 = t1*t2;
  
  // Zero division avoidance
  if (fabs(t1)<0.0001){
      t1 = 0.0001;}
  
  Hj << px/t2, py/t2, 0, 0,
       -py/t1, px/t1, 0, 0,
        py*(vx*py-vy*px)/t3, px*(vy*px-vx*py)/t3, px/t2, py/t2;
  return Hj;
}