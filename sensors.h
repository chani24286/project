#ifndef SENSOR_H
#define SENSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

class Sensor {
protected:
    double total_distance;
    double total_time;

public:
    Sensor();

    void setTotalDistance(double distance);
    void setTotalTime(double time);

    double getTotalDistance() const;
    double getTotalTime() const;
};

class LidarSensor : public Sensor {
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_scan, current_scan;
    ICP_out motion;
    bool isNoisey;
public:
    LidarSensor();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLatest_scan();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCurrent_scan();
    void setCurrent_scan(std::string path);
    ICP_out getMotion();
    void setMotion(ICP_out newMotion);
    void setIsNoisey(bool flag);
    bool getIsNoisy();
};
class GPSsensor : public Sensor {
private:
    double lastLat, lastLon, lat, lon;
public:
    GPSsensor();
    void setLastLatLon(double lastLat, double lastLon);
    double getLat();
    double getLon();
	void setLatAndLon(std::string path);
};
class pedometer : public Sensor {
private:
	double MFS; // meter for step
    int steps;
public:
	pedometer();
	void setMFS(double length, int numSteps);
	void setSteps(std::string path);
	double getMFS();
	double getSpeedness();
	double getStepsLength(double distance);
	int getSteps();
}

#endif // SENSOR_H
