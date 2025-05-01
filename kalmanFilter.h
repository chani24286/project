#pragma once
#pragma once
#include <Eigen/Dense>
#include <iostream>

class KalmanFilter {
public:
    KalmanFilter(double q_scale);

    void init(double t0, const Eigen::VectorXd& x0);
    void predict(double t);

    void updateLidar(const Eigen::VectorXd& z);
    void updateGPS(const Eigen::VectorXd& z);
    void updatePedometer(const Eigen::VectorXd& z);

    Eigen::VectorXd state() const;

private:
    double q; // scale of process noise
    double last_t;
    bool initialized;

    int n; // state size
    int m; // measurement size (not fixed)

    Eigen::VectorXd x; // state
    Eigen::MatrixXd P; // state covariance

    Eigen::MatrixXd A; // system dynamics
    Eigen::MatrixXd Q; // process noise

    Eigen::MatrixXd I; // identity
};
