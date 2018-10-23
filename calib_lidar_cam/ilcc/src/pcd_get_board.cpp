
#include <ros/ros.h>
#include <iostream>

#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>            /// sensor_msgs::PointCloud2
#include <pcl_conversions/pcl_conversions.h>    /// fromROSMsg toROSMsg

#include <pcl/filters/passthrough.h>

/// EuclideanCluster
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


#include <geometry_msgs/PointStamped.h>

#include <cmath>

using namespace std;

typedef pcl::PointXYZI myPoint;
typedef pcl::PointCloud<myPoint> myPointCloud;

ros::Publisher pubLaserROI;
ros::Publisher pubLaserBoard;

//min-segment param
//bouding box size for filter
float box_z_min = -1.0;
float box_z_max = +1.5;
float box_x_min = -0.5;
float box_x_max = +0.5;
float box_y_min = -2.0;
float box_y_max = +2.0;

bool save_board2pcd = true;
std::string pcd_save_path;
std::string save_file_name;

bool received_click_point = false;
myPoint click_point;

myPointCloud::Ptr cloudsetROI (myPointCloud::Ptr cloud, myPoint point)
{

  pcl::PassThrough<myPoint> pass;

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (point.z+box_z_min, point.z+box_z_max);// 0 1
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (point.x+box_x_min, point.x+box_x_max);// 0 1
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (point.y+box_y_min, point.y+box_y_max);// 0 1
  pass.filter (*cloud);

  return cloud;
}


myPointCloud::Ptr getPlane (myPointCloud::Ptr cloud)
{

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<myPoint> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.03);  // cm

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

//  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//                                      << coefficients->values[1] << " "
//                                      << coefficients->values[2] << " "
//                                      << coefficients->values[3] << std::endl;

//  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  myPointCloud::Ptr final (new myPointCloud);

  pcl::copyPointCloud<pcl::PointXYZI> (*cloud, *inliers, *final);    /// 取出所有的内点

  return final;
}


void point2txt(myPointCloud::Ptr cloud, std::string filename) {
  std::ofstream outfile(filename.c_str(), std::ios_base::trunc);

  myPoint temp;
  outfile << "#"<<cloud->size() << endl;
  for(unsigned int i = 0; i < cloud->size(); i++) {
      temp = cloud->points[i];

      outfile << temp.x << " " << temp.y << " "
              << temp.z << " " << temp.intensity << endl;
  }
  outfile.close();
}


void EuclideanCluster(pcl::PointCloud<myPoint>::Ptr cloud )
{

  myPoint point = click_point;

  double dw = 70.0;
  double dh = 80.0;
  double r  = point.x;
  double rate = 0.3;
  double N_theorey = dw/(2*r*sin(0.16/2)) * dh/(2*r*sin(1.33/2));
  cout << "N_theorey " << N_theorey << endl;


  cloud = cloudsetROI(cloud, point);

  sensor_msgs::PointCloud2 Ponitfilted2;
  pcl::toROSMsg(*cloud, Ponitfilted2);

  Ponitfilted2.header.stamp = Ponitfilted2.header.stamp;
  Ponitfilted2.header.frame_id = "/velodyne";
  pubLaserROI.publish(Ponitfilted2);


  pcl::search::KdTree<myPoint>::Ptr tree (new pcl::search::KdTree<myPoint>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<myPoint> ec;   //欧式聚类对象
  ec.setClusterTolerance (0.1);                     // 设置近邻搜索的搜索半径为2cm
  ec.setMinClusterSize (100);                 //设置一个聚类需要的最少的点数目为100
  ec.setMaxClusterSize (25000);               //设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod (tree);                    //设置点云的搜索机制
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中


  cout << cluster_indices.size() << " ";
  for(unsigned int i = 0; i < cluster_indices.size(); i++)
    cout << cluster_indices.at(i).indices.size() << " ";
  cout << endl;

  int maxcloudsize = cluster_indices.at(0).indices.size();
//  if(maxcloudsize > N_theorey*rate && maxcloudsize < N_theorey) {

    myPointCloud::Ptr final (new myPointCloud);
    pcl::copyPointCloud<myPoint> (*cloud, cluster_indices.at(0).indices, *final);    /// 取出所有的内点

    final = getPlane(final);    /// 取出平面
    cout << "plane size: " << final->size() << endl;

    sensor_msgs::PointCloud2 Ponitfilted;
    pcl::toROSMsg(*final, Ponitfilted);

    Ponitfilted.header.stamp = Ponitfilted.header.stamp;
    Ponitfilted.header.frame_id = "/velodyne";
    pubLaserBoard.publish(Ponitfilted);

    if(save_board2pcd) {
      pcl::io::savePCDFileASCII (pcd_save_path+save_file_name+".pcd", *final); //将点云保存到PCD文件中
      point2txt(final, pcd_save_path+save_file_name+".txt");
      ROS_INFO_STREAM(pcd_save_path);
    }else
    {
      ROS_INFO_STREAM("error");
    }
//  }

}

void region_growing_segmentation(pcl::PointCloud<myPoint>::Ptr cloud){

  std::cout << "normal_estimator.compute." << std::endl;
  pcl::search::Search<myPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<myPoint> >
                                                 (new pcl::search::KdTree<myPoint>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<myPoint, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (100);        //k nearest neighbors
  //normal_estimator.setRadiusSearch(0.2);//设置法相量的搜索半径
  normal_estimator.compute (*normals);

  myPoint point;
  point.x = 2.15;
  point.y = 0.16;
  point.z = 0.11;

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<myPoint> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (1.0, 2.5);//设置地面可能出现的区域，减少计算量
  pass.filter (*indices);

  std::cout << "reg.extract." << std::endl;
  pcl::RegionGrowing<myPoint, pcl::Normal> reg;
  reg.setMinClusterSize (5);
  reg.setMaxClusterSize (10000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);//这个参数变小，算法提速很多
  reg.setInputCloud (cloud);
//  reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (0.2);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);//这一步是计算 最慢

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "Max cluster has " << clusters[0].indices.size () << endl;

  int max_points_id = 0;

  for(unsigned int i=0; i<clusters.size (); i++)
  {
      if(clusters[max_points_id].indices.size() < clusters[i].indices.size())
          max_points_id = i;
  }

  myPointCloud::Ptr final (new myPointCloud);
  pcl::copyPointCloud<myPoint> (*cloud, clusters.at(max_points_id).indices, *final);    /// 取出所有的内点


  sensor_msgs::PointCloud2 Ponitfilted;
  pcl::toROSMsg(*final, Ponitfilted);

  Ponitfilted.header.stamp = Ponitfilted.header.stamp;
  Ponitfilted.header.frame_id = "/velodyne";
  pubLaserBoard.publish(Ponitfilted);

}

void callback_save(const sensor_msgs::PointCloud2ConstPtr& msg_lidar) {

  myPointCloud pointcloud;

  pcl::fromROSMsg(*msg_lidar, pointcloud);
  pcl::io::savePCDFileASCII ("/home/ha/PCL_pcd/test_pcd.pcd", pointcloud); //将点云保存到PCD文件中

  cout << "save to left_pcd.pcd size " << pointcloud.size() << endl;
}

void callback_lidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar) {

  myPointCloud pointcloud;

  pcl::fromROSMsg(*msg_lidar, pointcloud);

  if(received_click_point)
  {
      EuclideanCluster(pointcloud.makeShared());
  }

  //region_growing_segmentation(pointcloud.makeShared());
}


void ClickedPointHandler(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    myPoint point;

    cout<<"received a clicked point" << msg->point.x << "," << msg->point.y << "," << msg->point.z<<endl;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = msg->point.z;

    click_point = point;

    received_click_point = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_corners");
    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");
    nh_private.param<bool>("save_board2pcd", save_board2pcd, false);
    nh_private.param<string>("pcd_save_path", pcd_save_path, "/home/ha/");
    nh_private.param<string>("save_file_name", save_file_name, "chessboard");
    nh_private.param<float>("box_z_min", box_z_min, -1);
    nh_private.param<float>("box_z_max", box_z_max, +1.5);
    nh_private.param<float>("box_x_min", box_x_min, -0.5);
    nh_private.param<float>("box_x_max", box_x_max, +0.5);
    nh_private.param<float>("box_y_min", box_y_min, -1.0);
    nh_private.param<float>("box_y_max", box_y_max, +1.0);

    cout << box_z_min << " " << box_z_max << endl;
    cout << box_x_min << " " << box_x_max << endl;
    cout << box_y_min << " " << box_y_max << endl;

    cout << "save_board2pcd " << save_board2pcd << endl;
    cout << "pcd_save_path " << pcd_save_path << endl;

    pubLaserROI = nh.advertise<sensor_msgs::PointCloud2>("/LaserROI", 10);
    pubLaserBoard = nh.advertise<sensor_msgs::PointCloud2>("/LaserBoard", 10);



    ROS_INFO_STREAM("please public topic /click_point.....");

    ros::Subscriber sub_lidar = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1,
                                                                       callback_lidar);
//    ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 5,
//                                                                       callback_save);
    ros::Subscriber ClickedPointSub;
    ClickedPointSub = nh.subscribe<geometry_msgs::PointStamped>
                            ("/clicked_point", 100, ClickedPointHandler);



//    ros::spin(); /// 停在这儿了！！！！

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
