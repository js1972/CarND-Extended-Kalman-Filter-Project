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

    // first measurement
    cout << "EKF initialisation with first data row" << endl;

    // Create and initialise the state transition matrix F
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    // Create and initialise the covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    // Set the acceleration noise components
    // These values are probably a bit high but they help the rmse!
    noise_ax = 10;
    noise_ay = 10;

    // Measurement covariance
    //R_laser_ << 0.0225, 0,
    //            0,      0.0225;
    //R_radar_ << 0.0225, 0,      0,
    //            0,      0.0225, 0,
    //            0,      0,      0.0225;

    // New measurement covariance values as calculated by working out the variance
    // in the difference between x, y values and the ground truth values.
    R_radar_ << 0.01037, 0,           0,
                0,       1.06804e-06, 0,
                0,       0,           0.01129;

    R_laser_ << 0.00011, 0,
                0,       0.00010;

    
    // Laser measurement matrix
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // Radar measurement matrix
    Hj_ << 1, 1, 0, 0,
           1, 1, 0, 0,
           1, 1, 1, 1;

    previous_timestamp_ = measurement_pack.timestamp_;

    // Create and initialise the state vector with the first measurement
    ekf_.x_ = VectorXd(4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // First, convert radar from polar to cartesian coordinates
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rop = measurement_pack.raw_measurements_[2];

      ekf_.x_ << ro * cos(phi), ro * sin(phi), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // Done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Update the state transition matrix f according to the new elapsed time
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//expressed in seconds
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //if (dt > 0.0001) { THIS MAKES NO DIFF TO ACCURACY !!
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    // Set the process noise covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
               0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
               dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
               0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

    ekf_.Predict();
    previous_timestamp_ = measurement_pack.timestamp_;
  //}

  /*****************************************************************************
   *  Update
   ****************************************************************************/

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
