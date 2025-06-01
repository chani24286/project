#pragma once
#include <Eigen/Dense>
#include<optional>
#include "NevigationObject.h"
#include "sensors.h"
class KalmanFilter {
public:
    KalmanFilter(double q_scale);
    void init(double t0, const Eigen::VectorXd& x0);
    void predict(double t, NevigationObject &me);
    void updatePedometer(const Eigen::VectorXd& z, NevigationObject& me);
    void updateGPS(const Eigen::VectorXd& z, NevigationObject& me, GPSsensor sen);
    void updateLiDAR(const Eigen::VectorXd& z, NevigationObject &me);
    Eigen::VectorXd sensors(Eigen::MatrixXd H, const Eigen::MatrixXd R, const Eigen::VectorXd& z, NevigationObject& me);
    Eigen::VectorXd state() const;
    void updateMe(const Eigen::VectorXd& x, NevigationObject& me, std::optional<GPSsensor> sen);

private:
    int n;
    //q- noise , as much as i think that my model works well
    double q;
    double last_t;
    bool initialized;

    Eigen::VectorXd x;
    Eigen::MatrixXd P, A, Q, I;
};


void runKalmanFilter(GPSsensor& gps, pedometer& pedo, LidarSensor& lidar, NevigationObject& me);
