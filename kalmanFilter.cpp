#include "KalmanFilter.h"
#include "KalmanFilter.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <cmath>
#define M_PI 3.14159265358979323846

Eigen::VectorXd x;
using namespace std::chrono;

KalmanFilter::KalmanFilter(double q_scale)
    : q(q_scale), n(4), initialized(false) {
    x = Eigen::VectorXd::Zero(n);
    P = Eigen::MatrixXd::Identity(n, n);
    A = Eigen::MatrixXd::Identity(n, n);
    Q = Eigen::MatrixXd::Zero(n, n);
    I = Eigen::MatrixXd::Identity(n, n);
}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
    x = x0;
    last_t = t0;
    initialized = true;
}

void KalmanFilter::predict(double t) {
    if (!initialized) {
        std::cerr << "Filter not initialized!" << std::endl;
        return;
    }

    double dt = t - last_t;
    last_t = t;

    // Update A dynamically
    A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Update Q dynamically
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    Q << dt3 / 3, 0, dt2 / 2, 0,
        0, dt3 / 3, 0, dt2 / 2,
        dt2 / 2, 0, dt, 0,
        0, dt2 / 2, 0, dt;

    Q *= q;

    // Predict step
    x = A * x;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::updateLidar(const Eigen::VectorXd& z) {
    Eigen::MatrixXd H(2, 4);
    H << 1, 0, 0, 0,
        0, 1, 0, 0;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);

    Eigen::VectorXd y = z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    x = x + K * y;
    P = (I - K * H) * P;
}

void KalmanFilter::updateGPS(const Eigen::VectorXd& z) {
    // Same as LiDAR
    updateLidar(z);
}

void KalmanFilter::updatePedometer(const Eigen::VectorXd& z) {
    Eigen::MatrixXd H(2, 4);
    H << 0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);

    Eigen::VectorXd y = z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    x = x + K * y;
    P = (I - K * H) * P;
}

//Eigen::VectorXd KalmanFilter::state() const {
//    return x;
//}
Eigen::VectorXd kalman() {
    KalmanFilter kf(0.1);

    Eigen::VectorXd x0(4);
    x0 << 0, 0, 0, 0;
    kf.init(0.0, x0);

    auto start = steady_clock::now();

    while (true) {
        auto now = steady_clock::now();
        double t = duration<double>(now - start).count();

        kf.predict(t);

        // קריאת חיישני LiDAR בתדירות גבוהה (נניח כל 100 מ״ש)
        static double last_lidar_time = 0;
        if (t - last_lidar_time >= 0.1) {
            Eigen::VectorXd lidar(2);
            lidar << t, 0.5 * t;
            kf.updateLidar(lidar);
            last_lidar_time = t;
        }

        // קריאת GPS בתדירות נמוכה יותר (נניח כל שנייה)
        static double last_gps_time = 0;
        if (t - last_gps_time >= 1.0) {
            Eigen::VectorXd gps(2);
            gps << t + 0.2, 0.5 * t + 0.2;
            kf.updateGPS(gps);
            last_gps_time = t;
        }

        // קריאת מד צעדים (pedometer) גם כן בתדירות שונה
        static double last_pedo_time = 0;
        if (t - last_pedo_time >= 0.5) {
            Eigen::VectorXd pedo(2);
            pedo << 1.0, 0.5; // נגיד קצב קבוע של הליכה
            kf.updatePedometer(pedo);
            last_pedo_time = t;
        }

       // std::cout << "State at t=" << t << ": " << kf.state().transpose() << std::endl;

        std::this_thread::sleep_for(milliseconds(10));
    }

    return kf.state();
}

double computeHeadingAngle() {
    double vx = x(2);  // מהירות בציר X
    double vy = x(3);  // מהירות בציר Y

    double angle_rad = std::atan2(vy, vx); // זווית ברדיאנים
    double angle_deg = angle_rad * 180.0 / M_PI;

    // אם רוצים זווית בין 0 ל-360 מעלות:
    if (angle_deg < 0)
        angle_deg += 360.0;

    return angle_deg;
}


