
#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <pcl/common/common.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

typedef pcl::PointXYZI myPoint;
typedef pcl::PointCloud<myPoint> myPointCloud;


void hello_world(void)
{
  std::cout << "hello world\n";
}

/**
 * @brief toPointsXYZ
 * @param point_cloud
 * @return myPointXYZRID 转换为 PointXYZ
 */
pcl::PointCloud<pcl::PointXYZ>* toPointsXYZ(pcl::PointCloud<myPoint> point_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> *new_cloud = new pcl::PointCloud<pcl::PointXYZ>();
  for (pcl::PointCloud<myPoint>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {
    new_cloud->push_back(pcl::PointXYZ(pt->x, pt->y, pt->z));
  }
  return new_cloud;
}

myPointCloud filteredByIntensityDiff(myPointCloud point_cloud) {

  ///第一次数据 test
//  Eigen::Vector2d dx(-3, -1);
//  Eigen::Vector2d dy(1, 2);
//  Eigen::Vector2d dz(-1.1, 1);

  //// left_pcd
  Eigen::Vector2d dx(1.5, 2.5);
  Eigen::Vector2d dy(-0.5, 1);
  Eigen::Vector2d dz(-1, 1);


  myPointCloud filtered;

  for(myPointCloud::iterator pt = point_cloud.points.begin();
      pt < point_cloud.points.end(); pt++)
  {
      if(pt->x >= dx[0] && pt->x <= dx[1]
         && pt->y >= dy[0] && pt->y <= dy[1]
         && pt->z >= dz[0] && pt->z <= dz[1])
      {
          filtered.push_back( (*pt) );
      }
  }


//  pcl::io::savePCDFileASCII ("/home/ha/record/filtered.pcd", *(toPointsXYZ(filtered)));
  return filtered;
}


pcl::PointCloud <pcl::PointXYZRGB>::Ptr
region_growing_segment(myPointCloud::Ptr cloud) {

  pcl::search::Search<myPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<myPoint> > (new pcl::search::KdTree<myPoint>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<myPoint, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<myPoint> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<myPoint, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
//  int counter = 0;
//  while (counter < clusters[0].indices.size ())
//  {
//    std::cout << clusters[0].indices[counter] << ", ";
//    counter++;
//    if (counter % 10 == 0)
//      std::cout << std::endl;
//  }
//  std::cout << std::endl;


  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

  return colored_cloud;
//  pcl::visualization::CloudViewer viewer ("Cluster viewer");
//  viewer.showCloud(colored_cloud);
//  while (!viewer.wasStopped ())
//  {
//  }


//  sensor_msgs::PointCloud2 Ponitfilted;
//  pcl::toROSMsg(*colored_cloud, Ponitfilted);

//  Ponitfilted.header.stamp = Ponitfilted.header.stamp;
//  Ponitfilted.header.frame_id = "/velodyne";
//  pubLaserCloud.publish(Ponitfilted);

}


#endif
