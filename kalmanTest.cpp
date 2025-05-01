/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "kalman.h"

int main(int argc, char* argv[]) {

    // int n = 3; // Number of states
    int n = 4;
    // int m = 1; // Number of measurements
    int m = 3;

    double dt = 1.0; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(n, 1); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd RGPS(n, 2); // Measurement noise covariance GPS
    Eigen::MatrixXd HGPS(n, 2); // Measurement GPS
    Eigen::MatrixXd RLidar(n, 2); // Measurement noise covariance lidar
    Eigen::MatrixXd HLidar(n, 2); // Measurement lidar
    Eigen::MatrixXd RPedometer(n, 2); // Measurement noise covariance pedometer
    Eigen::MatrixXd HPedometer(n, 2); // Measurement pedometer
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
    C << 0, 0, 0, 0;

    // Reasonable covariance matrices
    Q << 0.01, 0, 0, 0,
        0, 0.01, 0, 0,
        0, 0, 0.01, 0,
        0, 0, 0, 0.01;

    HGPS << 1, 0, 0, 0,
        0, 1, 0, 0;

    RGPS << 1, 0, 0, 0,
        0, 1, 0, 0;

    HPedometer << 0, 0, 1, 0,
        0, 0, 0, 1;

    RPedometer << 1, 0, 0, 0,
        0, 1, 0, 0;
    RLidar << 1, 0, 0, 0,
        0, 1, 0, 0;
    HLidar << 1, 0, 0, 0,
        0, 1, 0, 0;

    P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

    // Construct the filter
    KalmanFilter kf(dt, A, C, Q, RLidar, HLidar, RPedometer, HPedometer, RGPS, HGPS, P);
    kf.init()

        // Best guess of initial states
        Eigen::VectorXd x0(n);
    double t = 0;
    x0 << 0, 0, 0, 0;
    kf.init(t, x0);

    // Feed measurements into filter, output estimated states
    //אמור לפעול בלולאה אינסופית
    for (int i = 0; i < 10; ++i) {
        //lidarMeasurement(0) = get data from LIDAR
        //icp(lastlidarMeasurments, lidarMeasurments);
        kf.update(I, lidarMeasurement, I);
        if (i % 10 == 0) {
            //gpsMeasurement(0) = get data from GPS
            kf.update(gpsMeasurement, I, I, time() - t0);
            //pedometerMeasurement(0) = get data from Pedometer
            kf.update(I, I, pedometerMeasurement, time() - t0);
        }

    }

    Eigen::VectorXd y(m);


    return 0;
}
