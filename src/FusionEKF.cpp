#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // measurement matrix H_
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // initial state matrix x_
  x_ = VectorXd(4);
  x_ << 1, 1, 1, 1;

  // initial transition matrix F
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

  // state covariance matrix P
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  // process covariance matrix Q
  Q_ = MatrixXd(4, 4);
  Q_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;


  ekf_ = KalmanFilter();
  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_, R_radar_);
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
    cout << "EKF: x_ init start" << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      rho = measurement_pack.raw_measurements_[0];
      phi = measurement_pack.raw_measurements_[1];

      ekf_.x_(0) = rho * cos(phi);
      ekf_.x_(1) = rho * sin(phi);
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    cout << "EKF: x_ init end" << endl;
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

   float noise_ax = 9.0;
   float noise_ay = 9.0;

   // compute elapsed time between measurements in seconds
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;

   float dt_2 = dt*dt;
   float dt_3 = dt_2*dt;
   float dt_4 = dt_3*dt;

   // Modify F matrix so time is integrated
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;

   // set the covariance matrix Q
   ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

   //cout << "EKF: Predict start" << endl;
   ekf_.Predict();
   //cout << "EKF: Predict end" << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //cout << "EKF: Update radar start" << endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    //cout << "EKF: Update radar end" << endl;
  } else {
    // Laser updates
    //cout << "EKF: Update lidar start" << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
    //cout << "EKF: Update lidar end" << endl;
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
