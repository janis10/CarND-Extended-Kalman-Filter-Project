#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    // Intialize the rmse
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // Argument checks
    if (estimations.size() != ground_truth.size()) {
        std::cout << "CalculateRMSE() - 'estimations' and 'ground_truth' sizes "
                     "do not match!"
                  << std::endl;
        return rmse;
    }
    if (estimations.size() == 0) {
        std::cout << "CalculateRMSE() - 'estimations' is empty!" << std::endl;
        return rmse;
    }

    // Accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        // Element-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    // Calculate the mean
    rmse = rmse / estimations.size();
    // Calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    // Assign values
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // Pre-compute a set of terms to avoid repeated calculation
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1 * c2);

    MatrixXd Hj(3, 4);
    // Check division by zero
    if (fabs(c1) < 0.0001) {
        std::cout << "CalculateJacobian() - Error - Division by Zero!"
                  << std::endl;
        Hj.fill(0.0);
        return Hj;
    }

    // Fill the Jacobian
    Hj << (px / c2), (py / c2), 0, 0, -(py / c1), (px / c1), 0, 0,
        py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2,
        py / c2;

    return Hj;
}
