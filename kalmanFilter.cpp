#include "kalmanFilter.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>
#include "NevigationObject.h"
#include "sensors.h"
#include <optional> 
#include "graph.h"

using namespace std::chrono;

KalmanFilter::KalmanFilter(double q_scale)
    : q(q_scale), n(2), initialized(false) {
    x = Eigen::VectorXd::Zero(n);
    P = Eigen::MatrixXd::Identity(n, n);
    A = Eigen::MatrixXd::Identity(n, n);
    Q = Eigen::MatrixXd::Zero(n, n);
    I = Eigen::MatrixXd::Identity(n, n);
}
void KalmanFilter:: updateMe(const Eigen::VectorXd& x, NevigationObject& me, std::optional<GPSsensor> sen=std::nullopt) {
    // Update the state with the current position

    if (x(0) <= 0) 
      me.goToNextEdge();

    if (x(0) <= 10 && sen.has_value()) {
        std::vector < Edge > trail= me.getTrail();
        if (trail.size() < 2) {
            me.setNextTurnAngle(0.0);
            return;
        }
        Node next = nodes[trail[1].to];
        double ang= calculate_angle(sen.value().getLat(), sen.value().getLon(),me.getNextNode().lat, me.getNextNode().lon, next.lat, next.lon);
        me.setNextTurnAngle(ang);
    }
    printing("kalman update position and valocity: " +std::to_string( x(0))+" "+ std::to_string(x(1)));
	me.setPosition(x(0));

}
void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
    x = x0;
    last_t = t0;
    initialized = true;
}

void KalmanFilter::predict(double t, NevigationObject &me) {
    if (!initialized) {
        std::cerr << "Filter not initialized!" << std::endl;
        return;
    }

    double dt = t - last_t;
    last_t = t;

    // Update A for distance and speed
    A << 1, dt,
        0, 1;

    // Update Q (process noise)
    double dt2 = dt * dt;
    Q << dt2 / 2, dt / 2,
        dt / 2, 1;
    Q *= q;

    x = A * x;
    P = A * P * A.transpose() + Q;
	updateMe(x, me);
}

void KalmanFilter::updatePedometer(const Eigen::VectorXd& z, NevigationObject& me) {
    Eigen::MatrixXd H(1, 2);
    H << 0, 1; // מודד רק את המהירות

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1); // רעש מדידה
    Eigen::VectorXd x = sensors(H, R, z, me);
    updateMe(x, me);
}
void KalmanFilter::updateLiDAR(const Eigen::VectorXd& z, NevigationObject& me) {
    Eigen::MatrixXd H(1, 2);
    H << 1, 0; 

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * 0.2; 
    Eigen::VectorXd x = sensors(H, R, z, me);
    updateMe(x, me);
}
void KalmanFilter::updateGPS(const Eigen::VectorXd& z, NevigationObject& me, GPSsensor sen) {
    Eigen::MatrixXd H(1, 2);
    H << 1, 0; // מודד את המרחק הכולל

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * 2.0; 
    Eigen::VectorXd x = sensors(H, R, z, me);
    updateMe(x, me, sen);
}
Eigen::VectorXd KalmanFilter::sensors(Eigen::MatrixXd H, const Eigen::MatrixXd R, const Eigen::VectorXd& z, NevigationObject& me) {
    Eigen::VectorXd y = z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    x = x + K * y;
    P = (I - K * H) * P;
    return x;
}

Eigen::VectorXd KalmanFilter::state() const {
    return x;
}

void runKalmanFilter(GPSsensor& gps, pedometer& pedo, LidarSensor& lidar, NevigationObject& me) {
    KalmanFilter kf(0.1);
    Eigen::VectorXd x0(2); // [distance, speed]
    x0 << 0.0, 0.0;
    kf.init(0.0, x0);

    auto start = steady_clock::now();

    for (int i = 0; i < 5; ++i) {
        auto now = steady_clock::now();
        double t = duration<double>(now - start).count();
        kf.predict(t, me);

		Eigen::VectorXd gpsVector(1);
		gpsVector << gps.getTotalDistance(); // distance from GPS
        kf.updateGPS(gpsVector, me, gps);
        //pedomete update
       
        Eigen::VectorXd pedometerVector(1);
        pedometerVector << pedo.getSpeedness(); // speed from pedometer
        kf.updatePedometer(pedometerVector, me);
    
        //lidar update
        Eigen::VectorXd lidarVector(1);
        lidarVector << lidar.getTotalDistance();
        kf.updateLiDAR(lidarVector, me);

        std::this_thread::sleep_for(milliseconds(50));
    }

}
