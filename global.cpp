#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#define M_PI 3.14159265358979323846
void translatePointCloud(
    const std::string& input_file,
    const std::string& output_file,
    float dx, float dy, float dz)
{
    // קריאה מהקובץ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the input file \n");
        return;
    }

    // מטריצת טרנספורמציה: זהות + הזזה
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << dx, dy, dz;

    // החלת הטרנספורמציה
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // שמירה לקובץ
    pcl::io::savePCDFileBinary(output_file, *transformed_cloud);
}
double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a = sin(dlat / 2) * sin(dlat / 2) +
        cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
        sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen(cmd, "r"), _pclose);
    if (!pipe) {
        throw std::runtime_error("popen failed");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}
