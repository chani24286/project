#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

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
