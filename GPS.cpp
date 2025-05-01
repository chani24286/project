#include "GPS.h"
#include <cmath>
#include <iostream>
#include <string>

#define M_PI 3.14159265358979323846

struct GPSData {
    double latitude;   // �� ����
    double longitude;  // �� ����
};

// ������ ������� ������� �� ������ ������
static double currentLatitude = 32.0853;   // ���� �� ����
static double currentLongitude = 34.7818;

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// ������� ������ ����� ��� ��� ������� �-GPS
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // ����� ���� ���� ����������
    const double R = 6371.0;

    // ���� ���� ����� ������ ��������
    double lat1Rad = degreesToRadians(lat1);
    double lon1Rad = degreesToRadians(lon1);
    double lat2Rad = degreesToRadians(lat2);
    double lon2Rad = degreesToRadians(lon2);

    // ����� �������
    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;

    // ����� ��� a
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
        std::cos(lat1Rad) * std::cos(lat2Rad) *
        std::sin(dLon / 2) * std::sin(dLon / 2);

    // ����� ��� c
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // ����� �����
    double distance = R * c;  // ����� ����������
    return distance;
}
GPSData getSimulatedGPS() {
    // ����� ����� ���� ��� �����
    currentLatitude += 0.0001;
    currentLongitude += 0.0001;

    return { currentLatitude, currentLongitude };
}
