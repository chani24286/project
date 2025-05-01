#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

void translatePointCloud(
    const std::string& input_file,
    const std::string& output_file,
    float dx, float dy, float dz)
{
    // ����� ������
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the input file \n");
        return;
    }

    // ������ �����������: ���� + ����
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << dx, dy, dz;

    // ���� ������������
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // ����� �����
    pcl::io::savePCDFileBinary(output_file, *transformed_cloud);
}
