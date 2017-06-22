#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

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
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht;
	S = S + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */ 
	//x is px,py,vx,vy 
	//x' is px+vx*dt, py+vy*dt, vx, vy ... given by F_
	//x is measured as px, py   ... given by H_
	//x is measured as sqrt(px*px+py*py), arctan(px/py), (px*vx+py*vy)/sqrt(px*px+py*py)   ... given by h_radar func	

	/*Tools tools; 
	VectorXd H_ = tools.CalculateJacobian(x_); 
	VectorXd z_pred = MatrixXd(3,1); // H_ * x_; //apply h function here, Hj matrix everywhere else
		float ro = sqrt(x_[0]*x_[0]+x_[1]*x_[1]); if (ro==0) ro=1;
		z_pred << ro, atan(x_[1]/x_[0]), (x_[0]*x_[2]+x_[1]*x_[3])/ro;
*/	VectorXd z_pred = hx_;

	VectorXd y = z - z_pred;
	float pi2 = 2*3.14159; y[1]/=pi2;
	y[1] -= round(y[1]); y[1]*=pi2;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
