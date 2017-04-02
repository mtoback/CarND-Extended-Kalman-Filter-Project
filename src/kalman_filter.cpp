#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_Laser_in, MatrixXd &R_Radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_Laser_ = R_Laser_in;
  R_Radar_ = R_Radar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	cout << " F_:" << F_.cols() << "," << F_.rows() << endl;
	std::cout << " P_:" << P_.cols() << "," << P_.rows() << std::endl;
	std::cout << " Ft:" << Ft.cols() << "," << Ft.rows() << std::endl;
	std::cout << " Q:" << Q_.cols() << "," << Q_.rows() << std::endl;
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
	MatrixXd PHt = P_ * Ht;
	MatrixXd S = H_ * PHt + R_Laser_;
	MatrixXd Si = S.inverse();
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
	// first calculate the jacobian from x_
	Tools tools;
	/**
	 *  calculate zpred
	 *  Convert the last cartesian estimate to polar
	 */
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];
	VectorXd z_pred(3);
	float c1 = sqrt(pow(px,2) + pow(py,2));
	float c2 = atan2(py,px);
	float c3 = (px*vx + py*vy)/c1;
	if ( c1 < 0.001 )
	   z_pred << 0.0, 0.0, 0.0;
	else
	{
		z_pred << c1, c2, c3;
	}
	VectorXd y = z - z_pred;
	// use the last state value to calculate jacobian, which is then used to update S, P and K
	MatrixXd Hj = tools.CalculateJacobian(x_);
	MatrixXd Ht = Hj.transpose();
	MatrixXd PHt = P_ * Ht;
	MatrixXd S = Hj * PHt + R_Radar_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj) * P_;

	//new estimate
	x_ = x_ + (K * y);
}
