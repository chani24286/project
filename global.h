
#include <string>

// ���� ����� �� ��� ������ ������ PCD ����� ���
void translatePointCloud(
    const std::string& input_file,
    const std::string& output_file,
    float dx, float dy, float dz);

// ���� ����� ��������
double haversine(double lat1, double lon1, double lat2, double lon2);

// ���� �� ����� ��� ��� ������ ��������� (lat/lon) ����������
double deg2rad(double degrees);
std::string exec(const char* cmd);


