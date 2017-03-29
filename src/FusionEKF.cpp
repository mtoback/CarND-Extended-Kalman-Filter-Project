#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * The Extended Kalman Filter (EKF) is the non-linear version of the Kalman Filter
 * It linearizes about an estimate of the current mean and covariance.
 *
 * Like the Kalman Filter, it cycles between accepting new inputs as to where the
 * object being tracked is (update), and updating the next estimated position of the object (predict)
 */
/*
 * Constructor
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  // R is the observation noise/covariance
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
   * H provides the coefficients to compare the current position
   * with where we think the object is. For linear objects, it contains
   * the position (x,y) and velocity (vx, vy) vs position and velocity
   *
   * For non-linear measurements like radar, we have polar coordinates
   * and so we have a Jacobian Matrix with three inputs (distance, angle,
   * and velocity along the distance line) vs [x, y, vx, vy] to convert it
   * to an estimated linear form
   */
  H_laser_ = MatrixXd(2, 4);
  /**
  H_laser_ << 1.0, 0,0, 0.0, 0.0,
		      0.0, 1.0, 0.0, 0.0;
  **/
  H_laser_(0,0) = 1.0;
  H_laser_(0,1) = 0.0;
  H_laser_(0,2) = 0.0;
  H_laser_(0,3) = 0.0;
  H_laser_(1,0) = 0.0;
  H_laser_(1,1) = 1.0;
  H_laser_(1,2) = 0.0;
  H_laser_(1,3) = 0.0;
  ekf_.H_ = H_laser_;


  /**
   * * TODO:
    * Set the process and measurement noises
  */
	//the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
   Q_ = MatrixXd(4, 4);
   //state covariance matrix P
   ekf_.P_ = MatrixXd(4, 4);
   ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;
   ekf_.R_Laser_ = R_laser_;
   ekf_.R_Radar_ = R_radar_;
   noise_ax_ = 9;
   noise_ay_ = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
	// first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	  Tools tools = Tools();
    	  Hj_ = tools.CalculateJacobian(ekf_.x_);
    	  VectorXd z = VectorXd(3);
    	  z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],
    			  measurement_pack.raw_measurements_[2];
		  cout << ekf_.x_ << endl;
		  ekf_.x_ = z.transpose() * Hj_;
		  cout << ekf_.x_ << endl;
		  cout << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		//set the state with the initial location and zero velocity
		ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

		previous_timestamp_ = measurement_pack.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	//1. Modify the F matrix so that the time is integrated
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;
	//2. Set the process covariance matrix Q
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << noise_ax_*pow(dt,4)/4.0, 0, noise_ax_*pow(dt,3)/2.0, 0,
			  0, noise_ay_*pow(dt,4)/4.0, 0, noise_ay_*pow(dt,3)/2.0,
			  noise_ax_*pow(dt,3)/2.0, 0, noise_ax_*pow(dt,2), 0,
			  0, noise_ay_*pow(dt,3)/2.0, 0, noise_ay_*pow(dt,2);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  cout << "sensor out: " << measurement_pack.sensor_type_ << endl;
  cout << "data out: " << measurement_pack.raw_measurements_ << endl;

  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Radar updates
	VectorXd z = VectorXd(2);
	z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
	ekf_.Update(z);
  } else {
    // Laser updates
		VectorXd z = VectorXd(3);
		z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],
				measurement_pack.raw_measurements_[2];
	ekf_.UpdateEKF(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

