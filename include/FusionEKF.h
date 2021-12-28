#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"
#include <fstream>
#include <string>
#include <vector>

class FusionEKF {
  public:
    /**
     * Constructor.
     */
    FusionEKF();

    /**
     * Destructor.
     */
    virtual ~FusionEKF();

    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter EKF_;

  private:
    // LiDAR covariance matrix
    Eigen::MatrixXd R_laser_;
    // RADAR covariance matrix
    Eigen::MatrixXd R_radar_;
    // LiDAR output matrix
    Eigen::MatrixXd C_laser_;
    // RADAR output matrix (after linearization)
    Eigen::MatrixXd Cj_;

    // Check whether the tracking toolbox was initialized or not
    bool is_initialized_;

    // Previous timestamp
    long long previous_timestamp_;

    // Acceleration noise
    float noise_ax_;
    float noise_ay_;

    // Tool object used to compute Jacobian and RMSE
    Tools tools;
};

#endif /* FusionEKF_H_ */
