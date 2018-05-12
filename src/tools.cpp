#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  //check the validity of the following inputs:
  //* the estimation vector size should not be zero
  //* the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
    ||estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); i++){
    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

 return rmse;
}

// Function to avoid division by zero
float Tools::DivisionCheck(const float & value, const float epsilon) {
  if (value < epsilon)
    return epsilon;
  else
    return value;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = DivisionCheck(px*px + py*py);
  float c2 = DivisionCheck(sqrt(c1));
  float c3 = DivisionCheck(c1 * c2);
  float d1 = vx*py - vy*px; 
  float d2 = px*vy - py*vx;

  //compute the Jacobian matrix
  MatrixXd Hj = MatrixXd(3, 4);
  Hj << (px/c2), (py/c2), 0, 0, 
         -(py/c1), (px/c1), 0, 0, 
         py*d1/c3, px*d2/c3, px/c2, py/c2;

  return Hj;
}
