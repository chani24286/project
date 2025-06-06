/**
* Kalman filter implementation using Eigen. Based on the following
* introductory paper:
*
*     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <Eigen/Dense>

#pragma once

class KalmanFilter {

public:

    /**
    * Create a Kalman filter with the specified matrices.
    *   A - System dynamics matrix
    *   C - Output matrix
    *   Q - Process noise covariance
    *   R - Measurement noise covariance
    *   P - Estimate error covariance
    */
    KalmanFilter::KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& RPedometer,
        const Eigen::MatrixXd& HPedometer,
        const Eigen::MatrixXd& RGPS,
        const Eigen::MatrixXd& HGPS,
        const Eigen::MatrixXd& HLidar,
        const Eigen::MatrixXd& RLidar,
        const Eigen::MatrixXd& P);

    /**
    * Create a blank estimator.
    */
    KalmanFilter();

    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(double t0, const Eigen::VectorXd& x0);



    /**
    * Update the estimated state based on measured values,
    * using the given time step and dynamics matrix.
    */
    void update(const Eigen::Vector2d& gpsMeasurement, const Eigen::Vector2d& lidarMeasurement,
        const Eigen::Vector2d& pedometerMeasurement);
    /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
    void update(const Eigen::Vector2d& gpsMeasurement,
        const Eigen::Vector2d& lidarMeasurement,
        const Eigen::Vector2d& pedometerMeasurement,
        double dt);

    void Sensor(const Eigen::VectorXd& y, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);

    /**
    * Return the current state and time.
    */
    Eigen::VectorXd state() { return x_hat; };
    double time() { return t; };

private:

    // Matrices for computation
    Eigen::MatrixXd A, C, Q, R, P, K, P0, HLidar, HGPS, HPedometer, RLidar, RGPS, RPedometer;

    // System dimensions
    int m, n;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt, sigma;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat, x_hat_new;
};
