#include "GPS.h"
#include <cmath>
#include <iostream>
#include <string>

#define M_PI 3.14159265358979323846

struct GPSData {
    double latitude;   // קו רוחב
    double longitude;  // קו אורך
};

// משתנים פנימיים שזוכרים את המיקום האחרון
static double currentLatitude = 32.0853;   // למשל תל אביב
static double currentLongitude = 34.7818;

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// פונקציה לחישוב המרחק בין שני מיקומים ב-GPS
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // רדיוס כדור הארץ בקילומטרים
    const double R = 6371.0;

    // המרת קווי הרוחב והאורך לרדיאנים
    double lat1Rad = degreesToRadians(lat1);
    double lon1Rad = degreesToRadians(lon1);
    double lat2Rad = degreesToRadians(lat2);
    double lon2Rad = degreesToRadians(lon2);

    // חישוב ההפרשים
    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;

    // חישוב ערך a
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
        std::cos(lat1Rad) * std::cos(lat2Rad) *
        std::sin(dLon / 2) * std::sin(dLon / 2);

    // חישוב ערך c
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // חישוב המרחק
    double distance = R * c;  // המרחק בקילומטרים
    return distance;
}
GPSData getSimulatedGPS() {
    // עושים תזוזה קטנה בכל קריאה
    currentLatitude += 0.0001;
    currentLongitude += 0.0001;

    return { currentLatitude, currentLongitude };
}
