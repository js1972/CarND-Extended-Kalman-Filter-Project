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
    cout << "EKF initialisation with first data row" << endl;

    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    // create the covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    //set the acceleration noise components
    noise_ax = 5;
    noise_ay = 5;

    //measurement covariance
    R_laser_ << 0.0225, 0,
                0,      0.0225;

    R_radar_ << 0.0225, 0,      0,
                0,      0.0225, 0,
                0,      0,      0.0225;

    // Laser measurement matrix
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // Radar measurement matrix
    Hj_ << 1, 1, 0, 0,
           1, 1, 0, 0,
           1, 1, 1, 1;

    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rop = measurement_pack.raw_measurements_[2];

      float x = ro * cos(phi);
      float y = ro * sin(phi);
      //if (x == 0 or y == 0) {
      //  return;
      //}

      ekf_.x_ << x, y, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout << "LASER measurement: x=" << measurement_pack.raw_measurements_[0] << "\ty=" << measurement_pack.raw_measurements_[1] << endl;
      //if (measurement_pack.raw_measurements_[0] == 0 or measurement_pack.raw_measurements_[1] == 0) {
      //  return;
      //}

      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    cout << "INITIAL STATE:" << endl << ekf_.x_ << endl;

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
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  if (dt > 0.0001) {
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    //set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
               0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
               dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
               0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

    ekf_.Predict();
    previous_timestamp_ = measurement_pack.timestamp_;
  }

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
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}
