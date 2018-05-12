#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  x_ = F_ * x_; //predict the new mean
  P_ = F_ * P_ * F_.transpose() + Q_; //predict the new covariance
  //cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl;

}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - (H_ * x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  //update the mean and covariance with the measured data
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  Tools tools;  //use to compute Jacobian later

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  //map the predicted position and speed to the polar coordinates of range, bearing and range rate.
  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px); // atan2() returns values between -pi and pi
  float rho_dot = (px*vx + py*vy) / rho; //range rate or polar velocity

  //std::cout << "PHI " << phi << " " << z(1) << " "<< std::endl;
	
  //normalizaing angles
  const float PI = 3.14159265;
  while( (phi-z(1)) > PI ) {
  	phi -= 2*PI;
  }
  while( (z(1)-phi) > PI ) {
  	phi += 2*PI;
  }

  VectorXd Hx = VectorXd(3);
  Hx << rho, phi, rho_dot;

  MatrixXd Hj_ = MatrixXd(3, 4);
  Hj_ = tools.CalculateJacobian(x_);
  MatrixXd Hjt = Hj_.transpose();

  VectorXd y = z - Hx;
  MatrixXd S = Hj_ * P_ * Hjt + R_;
  MatrixXd K = (P_ * Hjt) * S.inverse();

  //update the prediction mean and covariance with the measured data
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
}
