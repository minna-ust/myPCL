#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>

pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
int num = 0;

void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
    std::vector< int > indices;
    if (event.getPointsIndices(indices)==-1)
        return;

    for (int i = 0; i < indices.size(); ++i)
    {
        clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
    }

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*clicked_points_3d, minPt, maxPt);
    std::cout << minPt.x << " " << maxPt.x << "   "
        << minPt.y << " " << maxPt.y << "   "
        << minPt.z << " " << maxPt.z << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);

    std::stringstream ss;
    std::string cloudName;
    ss << num++;
    ss >> cloudName;
    cloudName += "_cloudName";

    viewer->addPointCloud(clicked_points_3d, red, cloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}

void usage()
{
    std::cout << "Usage: areaPicking -file ***.pcd" << std::endl;
}

int main(int argc, char **argv)
{
    std::string fileName;
    pcl::console::parse_argument(argc, argv, "-file", fileName);
    if (fileName == "")
    {
        usage();
        return -1;
    }
    std::cout << "loadPCDFile: " << fileName << std::endl;
    if (pcl::io::loadPCDFile(fileName, *originalCloud))
    {
        std::cerr << "ERROR: Cannot open file " << std::endl;
        return -1;
    }

    cloud->header = originalCloud->header;
    cloud->points.resize(originalCloud->points.size());
    cloud->sensor_origin_ = originalCloud->sensor_origin_;
    cloud->sensor_orientation_ = originalCloud->sensor_orientation_;

    size_t j = 0;
    for (size_t i = 0; i < originalCloud->points.size(); i++)
    {
        if (!std::isfinite(originalCloud->points[i].x) ||
            !std::isfinite(originalCloud->points[i].y) ||
            !std::isfinite(originalCloud->points[i].z))
        {
            continue;
        }
        cloud->points[j] = originalCloud->points[i];
        j++;
    }
    viewer->addCoordinateSystem(1.0);
    viewer->addPointCloud(cloud, "bunny");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}
