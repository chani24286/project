
#include <string>

// מבצע תרגום של ענן נקודות בפורמט PCD לקובץ חדש
void translatePointCloud(
    const std::string& input_file,
    const std::string& output_file,
    float dx, float dy, float dz);

// ממיר מעלות לרדיאנים
double haversine(double lat1, double lon1, double lat2, double lon2);

// מחשב את המרחק בין שתי נקודות גאוגרפיות (lat/lon) בקילומטרים
double deg2rad(double degrees);
std::string exec(const char* cmd);


