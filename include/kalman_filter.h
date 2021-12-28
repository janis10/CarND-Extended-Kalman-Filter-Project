#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
  public:
    // State vector
    Eigen::VectorXd x_;
    // State covariance matrix
    Eigen::MatrixXd P_;
    // State transition matrix
    Eigen::MatrixXd A_;
    // Process covariance matrix
    Eigen::MatrixXd Q_;
    // Measurement matrix
    Eigen::MatrixXd C_;
    // Measurement covariance matrix
    Eigen::MatrixXd R_;

    /**
     * Constructor
     */
    KalmanFilter();

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param A_in Transition matrix
     * @param C_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in,
              Eigen::MatrixXd &A_in, Eigen::MatrixXd &C_in,
              Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k-1 and k in s
     */
    void Predict();

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param y The measurement at k
     */
    void UpdateKF(const Eigen::VectorXd &y);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param y The measurement at k
     */
    void UpdateEKF(const Eigen::VectorXd &y);

    /**
     * Updates the state estimate by using the residual z_k = y_k - H_k x_k
     * @param z The residual at k
     */
    void UpdateEstimates(const Eigen::VectorXd &z);
};

#endif /* KALMAN_FILTER_H_ */
