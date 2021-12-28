#include "FusionEKF.h"
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    // Hint: use noise_ax_ = 9 and noise_ay_ = 9 for your Q matrix.
    noise_ax_ = 9;
    noise_ay_ = 9;

    // Initializations
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // Measurement covariance matrix - LIDAR
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0, 0, 0.0225;
    // Measurement covariance matrix - RADAR
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;
    // Measurement matrix - LIDAR
    C_laser_ = MatrixXd(2, 4);
    C_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;
    // Measurement matrix - RADAR
    Cj_ = MatrixXd(3, 4);
    Cj_ << 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1;

    // Initialize transition matrix A_ in EKF instance EKF_
    EKF_.A_ = MatrixXd(4, 4);
    EKF_.A_ << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

    // State covariance matrix P
    EKF_.P_ = MatrixXd(4, 4);
    EKF_.P_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

    // Initialization:
    // sets up initial estimate based on the first measurement that arrives.
    if (!is_initialized_) {
        // At the initialization step, the state is initialized using the first
        // measurement.
        cout << "EKF: " << endl;
        EKF_.x_ = VectorXd(4);
        EKF_.x_ << 0, 0, 0, 0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // If measurement is RADAR: convert from polar to cartesian coords.
            float rho = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
            float dot_rho = measurement_pack.raw_measurements_(2);
            // Initialize state
            EKF_.x_(0) = rho * cos(phi);
            EKF_.x_(1) = rho * sin(phi);
            EKF_.x_(2) = dot_rho * cos(phi);
            EKF_.x_(3) = dot_rho * sin(phi);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // If measurement is LiDAR.
            // Initialize state: here only position is initialized
            EKF_.x_(0) = measurement_pack.raw_measurements_(0);
            EKF_.x_(1) = measurement_pack.raw_measurements_(1);
            EKF_.x_(2) = 0.0;
            EKF_.x_(3) = 0.0;
        }
        // Update timestamp
        previous_timestamp_ = measurement_pack.timestamp_;
        // Done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    // Prediction:
    // updates the state transition matrix Α according to the new
    // elapsed time dt, and the process noise covariance matrix.
    // dt = (curr_timestamp - previous_timestamp) / 1000000 (in seconds)
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    // Update previous_timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // Threshold dt > 0.001 for update
    if (dt > 0.001) {
        // Recall: A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]
        // that is, perform integration essentially
        EKF_.A_(0, 2) = dt;
        EKF_.A_(1, 3) = dt;

        // Update covariance matrix Q of state
        float dt_2 = dt * dt;
        float dt_3 = dt_2 * dt;
        float dt_4 = dt_3 * dt;
        EKF_.Q_ = MatrixXd(4, 4);
        EKF_.Q_ << dt_4 / 4 * noise_ax_, 0, dt_3 / 2 * noise_ax_, 0, 0,
            dt_4 / 4 * noise_ay_, 0, dt_3 / 2 * noise_ay_, dt_3 / 2 * noise_ax_,
            0, dt_2 * noise_ax_, 0, 0, dt_3 / 2 * noise_ay_, 0,
            dt_2 * noise_ay_;

        // Call prediction function
        EKF_.Predict();
    }

    // Update:
    // updates the state and covariance matrices based on the incoming sensor
    // measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // If measurement is RADAR.
        // Use Jacobian since the output function is non-linear
        Tools tools;
        Cj_ = tools.CalculateJacobian(EKF_.x_);
        EKF_.C_ = Cj_;
        EKF_.R_ = R_radar_;
        EKF_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // If measurement is LiDAR.
        EKF_.C_ = C_laser_;
        EKF_.R_ = R_laser_;
        EKF_.UpdateKF(measurement_pack.raw_measurements_);
    }

    // Print the estimated state and covariance
    cout << "x_ = " << EKF_.x_ << endl;
    cout << "P_ = " << EKF_.P_ << endl;
}
