#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void log(std::string msg, VectorXd& v) {
	return;
	std::cout << msg << "v=(" ;
	for (int i=0; i<v.rows(); i++)
		cout<< v[i]<<",";
	std::cout << ")\t" ;
}
void log(std::string msg, MatrixXd& p) {
	return;
	std::cout << msg << "P=((" ;
	for (int i=0; i<p.rows(); i++) {
	  for (int j=0; j<p.cols(); j++)
		cout<< p(i,j)<<",";
	  cout<< "), (";
	}
	std::cout << "))" << endl;
}

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  /**   Initialize State Prediction  */
	//create a 4D state vector, we don't know yet the values of the x state
	ekf_.x_ = VectorXd(4);

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd::Identity(4, 4); //for dt=0

	//set the acceleration noise components
	ekf_.Q_ = MatrixXd::Identity(4,4);	// to override
	
  /**   Initialize Measurement matrices  */
	//measurement & covariance matrix - laser
	H_laser_ = MatrixXd(2, 4);
	H_laser_ << 1, 0, 0, 0,
			   0, 1, 0, 0;
	ekf_.H_ = MatrixXd::Identity(4, 4);
	R_laser_ = MatrixXd(2, 2);
	R_laser_ << 0.0225, 0,
			  0, 0.0225;

  /**   Initialize Radar Measurement  */
  //measurement & covariance matrix - radar
  Hj_ = MatrixXd(3, 4);
  R_radar_ = MatrixXd(3, 3);
  R_radar_ <<  0.09, 0, 0,
					0, 0.0009, 0,
					0, 0, 0.09;
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

    // Create process covariance matrix
    ekf_.Q_ = Eigen::MatrixXd(4,4);
	ekf_.hx_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		//set the state with the initial measurement_pack.raw_measurements_ is VectorXd(3)  << ro,theta, ro_dot
		float rho = measurement_pack.raw_measurements_[0];
		float theta= measurement_pack.raw_measurements_[1];
		float ro_dot = measurement_pack.raw_measurements_[2];
		ekf_.x_ << rho*cos(theta), rho*sin(theta), ro_dot, ro_dot;	  
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		//set the state with the initial location and zero velocity
		//measurement_pack.raw_measurements_ is VectorXd(2)  << px,py
		ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
	log("Initialized ", ekf_.x_); log("Initialized ", ekf_.P_);
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

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

    // TODO: YOUR CODE HERE
	//1. Modify the F matrix so that the time is integrated
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	//2. Set the process covariance matrix Q
	float noise_ax = 9;
	float noise_ay = 9;
	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
   
   
	ekf_.Predict();
	log("PREDICTED ", ekf_.x_); log("PREDICTED ", ekf_.P_);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
//  log("Updating with RADAR " , ekf_.x_); std::cout << measurement_pack.raw_measurements_ << endl;
	// Radar updates	  
      ekf_.hx_ = VectorXd(3);
      
      float px = ekf_.x_[0]; 	if (px==0) px=0.0001;
      float py = ekf_.x_[1];
      float vx = ekf_.x_[2];
      float vy = ekf_.x_[3];

	float rho = sqrt(px*px + py*py); 
	float phi = atan2(py,px); //  arc tangent of y/x, in the interval [-pi,+pi] radians.
	float rhodot = (px*vx + py*vy) /rho;

	ekf_.hx_ << rho, phi, rhodot;
    // set H_ to Hj when updating with a radar measurement
    Hj_ = tools.CalculateJacobian(ekf_.x_);	
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	//std::cout << "Laser updated " << ekf_.x_ <<endl;
	log("UPDATED RADAR ", ekf_.x_); log("UPDATED RADAR ", ekf_.P_);

	} else {
//  log("Updating with LIDAR " , ekf_.x_); std::cout << measurement_pack.raw_measurements_ << endl;
    // Laser updates
	ekf_.H_ = H_laser_;
	//std::cout << "Copying R_ " << ekf_.R_ << " to " << endl << R_laser_ << endl;
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
	//std::cout << "Laser updated " << ekf_.x_ <<endl;	
	log("UPDATED LIDAR ", ekf_.x_); log("UPDATED LIDAR ", ekf_.P_);
  }

  // print the output
  log("Final ", ekf_.x_); // cout << "x_ = " << ekf_.x_ << endl;
  log("Final ", ekf_.P_); // cout << "P_ = " << ekf_.P_ << endl;
}
