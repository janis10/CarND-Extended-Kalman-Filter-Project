#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Control system:
// State evolution:     x+ = A x + B u + w, w ~ N(0, Q)
// Output evolution:    y = C x + v, v ~ N(0, R)
KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

// Initializa Kalman Filter instance.
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &A_in,
                        MatrixXd &C_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    A_ = A_in;
    C_ = C_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
     * Predict the state
     */
    // x_{k|k-1} = A x_{k-1|k-1}
    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::UpdateKF(const VectorXd &y) {
    /**
     * Update the state by using Kalman Filter equations for LIDAR.
     * LIDAR_measurement = (px, py).
     */
    // y_{k} = C x_{k|k-1}
    VectorXd y_pred = C_ * x_;
    // Residual of {actual measurement - predicted measurement}
    VectorXd z = y - y_pred;

    UpdateEstimates(z);
}

void KalmanFilter::UpdateEKF(const VectorXd &y) {
    /**
     * Update the state by using Extended Kalman Filter equations for RADAR.
     * RADAR_measurement:
     * (rho,phi,dot(rho)) = (sqrt(px^2+py^2),arctan(py/px),(px*vx+py*vy)/rho).
     */

    // Assign values from current state estimate.
    float px = x_[0], py = x_[1], vx = x_[2], vy = x_[3];

    // If both px and py are 0, then rho = 0 and we cannot divide.
    if (px == 0 && py == 0)
        return;

    float rho = sqrt(px * px + py * py), phi = atan2(py, px);

    // Checking the value is not zero
    if (rho < 0.0001)
        rho = 0.0001;
    float dot_rho = (px * vx + py * vy) / rho;

    // h(x) = (rho, phi, dot_rho)
    VectorXd h = VectorXd(3);
    h << rho, phi, dot_rho;
    VectorXd z = y - h;

    // Normalize the angle between -pi to pi
    while (z[1] < -M_PI)
        z[1] += 2 * M_PI;
    while (z[1] > M_PI)
        z[1] -= 2 * M_PI;

    UpdateEstimates(z);
}

void KalmanFilter::UpdateEstimates(const VectorXd &z) {
    // Construct matrices S, K.
    MatrixXd S = C_ * P_ * C_.transpose() + R_;
    MatrixXd K = P_ * C_.transpose() * S.inverse();
    // Update state estimate
    x_ = x_ + K * z;
    // Update covariance estimate
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * C_) * P_;
}
