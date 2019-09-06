#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <iostream>
#include <vector>
#include <ctime>
#include <iostream>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <quanergy/common/point_xyzir.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/intersections.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/project_inliers.h>
#include <quanergy/common/point_xyzir.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <functional>
#include <sstream>
#include "PartialCloud.hpp"

typedef quanergy::PointXYZIR PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::mutex cloud_mutex;
pcl::visualization::PCLVisualizer::Ptr viewer_2(new pcl::visualization::PCLVisualizer("PCD View1234"));

pcl::visualization::PCLVisualizer::Ptr viewer_(new pcl::visualization::PCLVisualizer("PCD View123"));

PointCloudT::Ptr clicked_points_3d(new PointCloudT);

// mViewer->registerPointPickingCallback(pointPickingCallback, (void*)&mCallbackArgs);

struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


void ppCallback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args *data = (struct callback_args *)args;

    int idx = event.getPointIndex();
    if (idx == -1)
        return;
    PointT pickedPt;
    event.getPoint(pickedPt.x, pickedPt.y, pickedPt.z);
    printf("Picked point with index %d, and coordinates %f, %f, %f.\n", idx, pickedPt.x, pickedPt.y, pickedPt.z);
    data->clicked_points_3d->points.push_back(pickedPt);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 230, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::stringstream ss;
    ss << "sphere_" << idx;
    // viewer_2->addSphere(pickedPt, 0.001, 0.5, 0.5, 0.1, ss.str());
}

std::function<void (const pcl::visualization::PointPickingEvent&, void*)> Functional = ppCallback;

int
main (int argc, char** argv)
{

  cv::CommandLineParser parser(argc, argv,
                               "{@input||}");

  std::string filename;
  if (parser.has("@input"))
  {
      filename = parser.get<std::string>("@input");
  }
  std::cout << "filename: " << filename << std::endl;

    {
        viewer_2->setBackgroundColor(0, 0, 0);
        viewer_2->addCoordinateSystem(1.0);
        viewer_2->initCameraParameters();
    }

    pcl::PointCloud<quanergy::PointXYZIR>::Ptr newCloud(new pcl::PointCloud<quanergy::PointXYZIR>);
    // Fill in the cloud data
    std::cout << "1 pcd file: " << filename << std::endl;
    pcl::io::loadPCDFile(filename.c_str(), *newCloud);
    std::cout << "PointCloud before filtering: "
              << newCloud->width * newCloud->height
              << " data points." << std::endl;

    for (size_t i = 0; i < 10; i++)
    {
        std::cout << i << "   " << newCloud->points[i].x << " "
                  << newCloud->points[i].y << " "
                  << newCloud->points[i].z << " "
                  << newCloud->points[i].intensity << " "
                  << newCloud->points[i].ring << std::endl;
    }

    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing0(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing1(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing2(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing3(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing4(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing5(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing6(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing7(new pcl::PointCloud<quanergy::PointXYZIR>);

    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing0NanRemoved(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing1NanRemoved(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing2NanRemoved(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing3NanRemoved(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing4NanRemoved(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing5NanRemoved(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing6NanRemoved(new pcl::PointCloud<quanergy::PointXYZIR>);
    pcl::PointCloud<quanergy::PointXYZIR>::Ptr cloudRing7NanRemoved(new pcl::PointCloud<quanergy::PointXYZIR>);

    for (size_t i = 0; i < newCloud->points.size(); i++)
    {
        quanergy::PointXYZIR point = newCloud->points[i];
        int ringIndex = point.ring;

        switch (ringIndex) {
            case 0:
                cloudRing0->push_back(point);
                break;
            case 1:
                cloudRing1->push_back(point);
                break;
            case 2:
                cloudRing2->push_back(point);
                break;
            case 3:
                cloudRing3->push_back(point);
                break;
            case 4:
                cloudRing4->push_back(point);
                break;
            case 5:
                cloudRing5->push_back(point);
                break;
            case 6:
                cloudRing6->push_back(point);
                break;
            case 7:
                cloudRing7->push_back(point);
                break;
            default:
                printf("Invalid point with ring: %d", point.ring);
        }
    }

    pcl::visualization::PointCloudColorHandlerCustom<quanergy::PointXYZIR> color0(cloudRing0, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<quanergy::PointXYZIR> color1(cloudRing1, 255, 0, 179);
    pcl::visualization::PointCloudColorHandlerCustom<quanergy::PointXYZIR> color2(cloudRing2, 188, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<quanergy::PointXYZIR> color3(cloudRing3, 51, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<quanergy::PointXYZIR> color4(cloudRing4, 0, 171, 255);
    pcl::visualization::PointCloudColorHandlerCustom<quanergy::PointXYZIR> color5(cloudRing5, 0, 255, 179);
    pcl::visualization::PointCloudColorHandlerCustom<quanergy::PointXYZIR> color6(cloudRing6, 34, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<quanergy::PointXYZIR> color7(cloudRing7, 247, 255, 0);



    // viewer_2->addPointCloud<quanergy::PointXYZIR>(cloudRing0, color0, "quanergy cloud ring 0");
    // viewer_2->addPointCloud<quanergy::PointXYZIR>(cloudRing1, color1, "quanergy cloud ring 1");
    // viewer_2->addPointCloud<quanergy::PointXYZIR>(cloudRing2, color2, "quanergy cloud ring 2");
    // viewer_2->addPointCloud<quanergy::PointXYZIR>(cloudRing3, color3, "quanergy cloud ring 3");
    // viewer_2->addPointCloud<quanergy::PointXYZIR>(cloudRing4, color4, "quanergy cloud ring 4");
    // viewer_2->addPointCloud<quanergy::PointXYZIR>(cloudRing5, color5, "quanergy cloud ring 5");
    // viewer_2->addPointCloud<quanergy::PointXYZIR>(cloudRing6, color6, "quanergy cloud ring 6");
    // viewer_2->addPointCloud<quanergy::PointXYZIR>(cloudRing7, color7, "quanergy cloud ring 7");

    if (cloudRing0->is_dense)
    {
        printf("****************\n");
    }
    // for (size_t i = 0; i < cloudRing0->points.size(); i++)
    // {
    //     std::cout << i << "   " << cloudRing0->points[i].x << " "
    //               << cloudRing0->points[i].y << " "
    //               << cloudRing0->points[i].z << " "
    //               << cloudRing0->points[i].intensity << " "
    //               << cloudRing0->points[i].ring << std::endl;
    // }
  pcl::KdTreeFLANN<quanergy::PointXYZIR> kdtree;

  std::cout << "size: " << cloudRing0->points.size() << std::endl;
  std::vector<int> indices;
  // pcl::removeNaNFromPointCloud(*cloudRing0, *cloudRing0NanRemoved, indices);
  cloudRing0NanRemoved->header = cloudRing0->header;
  cloudRing0NanRemoved->points.resize(cloudRing0->points.size());
  cloudRing0NanRemoved->sensor_origin_ = cloudRing0->sensor_origin_;
  cloudRing0NanRemoved->sensor_orientation_ = cloudRing0->sensor_orientation_;
  size_t j = 0;
  for (size_t i = 0; i < cloudRing0->points.size(); ++i)
  {
      // std::cout << i << "  " << cloudRing0->points[i].x << cloudRing0->points[i].y
          // << cloudRing0->points[i].z << std::endl;
      if (!std::isfinite(cloudRing0->points[i].x) ||
          !std::isfinite(cloudRing0->points[i].y) ||
          !std::isfinite(cloudRing0->points[i].z))
      {
          // printf("point is infinite!\n");
          continue;
      }
      // std::cout << "*****debug" << std::endl;
      cloudRing0NanRemoved->points[j] = cloudRing0->points[i];
      j++;
  }
  std::cout << "size2: " << cloudRing0NanRemoved->points.size () << std::endl;

  pcl::PointCloud<PointT>::Ptr cloudRing0NanRemovedBak = boost::make_shared<pcl::PointCloud<PointT>>();
  pcl::copyPointCloud(*cloudRing0NanRemoved, *cloudRing0NanRemovedBak);
  pcl::PassThrough<quanergy::PointXYZIR> pass;
  pass.setInputCloud(cloudRing0NanRemoved);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.9, 1.2);
  pass.filter(*cloudRing0NanRemoved);
  pass.setInputCloud(cloudRing0NanRemoved);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0.2, 0.5);
  pass.filter(*cloudRing0NanRemoved);

  std::sort(cloudRing0NanRemoved->points.begin(), cloudRing0NanRemoved->points.end(),
            [](const quanergy::PointXYZIR &point1, const quanergy::PointXYZIR &point2)
            {
                if (point1.y <= point2.y)
                    return true;
                return false;
            });
  pcl::PointCloud<quanergy::PointXYZIR>::Ptr pointsYMin30(new pcl::PointCloud<quanergy::PointXYZIR>);
  // 100pointsYMin->points.resize(100);
  float yValue = 0.0;
  for (size_t i = 0; i < 30; i++)
  {
      std::cout << i << "   " << cloudRing0NanRemoved->points[i].x << " " << cloudRing0NanRemoved->points[i].y
                << " " << cloudRing0NanRemoved->points[i].z << std::endl;
      pointsYMin30->points.push_back(cloudRing0NanRemoved->points[i]);
      yValue += cloudRing0NanRemoved->points[i].y;
  }
  std::cout << "yValue for pointsYMin30: " << yValue / 30 << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer_3(new pcl::visualization::PCLVisualizer("debug"));
  {
      viewer_3->setBackgroundColor(0, 0, 0);
      viewer_3->addCoordinateSystem(1.0);
      viewer_3->initCameraParameters();
  }
  pcl::visualization::PointCloudColorHandlerCustom<PointT> colorDebug(cloudRing0NanRemovedBak, 239, 0, 255);
  viewer_3->addPointCloud(cloudRing0NanRemovedBak, colorDebug, "debug");

  PointT yMinCenter;
  boost::shared_ptr<PartialCloud<quanergy::PointXYZIR>> pcYMin = boost::make_shared<PartialCloud<quanergy::PointXYZIR>>(cloudRing0NanRemovedBak, 0.9, 1.2, 0.2, 0.5);
  yMinCenter = pcYMin->getYMinCenter(10);
  std::cout << "y min center: " << yMinCenter.x << " " << yMinCenter.y << " " << yMinCenter.z << std::endl;
  viewer_2->addSphere(yMinCenter, 0.005, 0.5, 0.5, 0.1, "y min center");


  PointT yMaxCenter;
  boost::shared_ptr<PartialCloud<quanergy::PointXYZIR>> pc = boost::make_shared<PartialCloud<quanergy::PointXYZIR>>(cloudRing0NanRemovedBak, 1.04, 1.06, -0.15, 0);
  yMaxCenter = pc->getYMaxCenter(5);
  std::cout << "y max 10 center: " << yMaxCenter.x << yMaxCenter.y << yMaxCenter.z << std::endl;
  viewer_2->addSphere(yMaxCenter, 0.005, 0.5, 0.5, 0.1, "y max center");
  pcl::visualization::PointCloudColorHandlerCustom<PointT> green2(pc->mYMaxCloud, 255, 137, 0);
  viewer_2->addPointCloud(pc->mYMaxCloud, green2, "y max cloud");
  viewer_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "y max cloud");
  pcl::visualization::PointCloudColorHandlerCustom<PointT> colorDebug2(pc->mCloudIn, 255, 137, 0);

  viewer_2->addPointCloud(pc->mCloudIn, colorDebug2, "cloudring0");




  std::sort(cloudRing0NanRemoved->points.begin(), cloudRing0NanRemoved->points.end(),
            [](const quanergy::PointXYZIR &point1, const quanergy::PointXYZIR &point2)
            {
                if (point1.x >= point2.x)
                    return true;
                return false;
            });

  // for (size_t i = 0; i < cloudRing0NanRemoved->points.size(); i++)
  // {
  //     std::cout << i << "   " << cloudRing0NanRemoved->points[i].x << cloudRing0NanRemoved->points[i].y
  //         << cloudRing0NanRemoved->points[i].z << std::endl;
  // }
  kdtree.setInputCloud(cloudRing0NanRemoved);

  quanergy::PointXYZIR searchPoint;

  // K nearest neighbor search

  int K = 10;

  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 0.01;

  std::cout << "Neighbors within radius search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;

  PointCloudT::Ptr illegalPoints(new PointCloudT);
  for (size_t i = 0; i < cloudRing0NanRemoved->points.size(); ++i)
  {
      searchPoint = cloudRing0NanRemoved->points[i];

      if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < 2)
      {
          illegalPoints->points.push_back(searchPoint);
      }
  }

  pcl::visualization::PointCloudColorHandlerCustom<PointT> green(illegalPoints, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(cloudRing0NanRemoved, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red(pointsYMin30, 255, 0, 0);
  cloud_mutex.lock();
  struct callback_args cbArgs;
  PointCloudT::Ptr clicked_points_3d(new PointCloudT);
  cbArgs.clicked_points_3d = clicked_points_3d;
  cbArgs.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer_2);
  viewer_2->registerPointPickingCallback(ppCallback, (void *)&cbArgs);
  viewer_2->addPointCloud(cloudRing0NanRemoved, blue,  "cloud ring0");
  viewer_2->addPointCloud(illegalPoints, green, "illegal points");
  viewer_2->addPointCloud(pointsYMin30, red, "30 points y min");
  viewer_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "illegal points");
  viewer_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "30 points y min");

  cloud_mutex.unlock();

  while (!viewer_2->wasStopped())
  {
      viewer_2->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }


  // if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  // {
  //   for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
  //     std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
  //               << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
  //               << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
  //               << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  // }


  return 0;
}
