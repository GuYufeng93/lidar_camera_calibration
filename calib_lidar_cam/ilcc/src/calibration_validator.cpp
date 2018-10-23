#include "ros/ros.h"
#include <sstream>
#include <cmath>


#include "ilcc/utils.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Image.h>       /// sensor_msgs::Image

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/common/transforms.h>  /// pcl::transformPointCloud
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>

std::string image_topic, save_path, save_image_name, corner_file_name;

Eigen::Isometry3f processImage(cv::Mat image, std::vector<cv::Point3f>& bBoardCorner) {

    cv::Size imageSize = cv::Size(image.cols, image.rows);
    cv::Mat M1l, M2l;     // 映射表
    cv::Mat R_l;

    cv::Mat camK, distort_param;

    cv::FileStorage fs;
    fs.open("/home/ha/PCL_pcd/monostereo.yaml", cv::FileStorage::READ);
    fs["Kr"] >> camK;
    fs["dr"] >> distort_param;
    fs.release();

    cv::initUndistortRectifyMap(camK, distort_param, R_l, camK, imageSize, CV_32F, M1l, M2l);
    cv::Mat rectifyImageL;
    cv::remap(image, rectifyImageL, M1l, M2l, cv::INTER_LINEAR);

    cv::Size boardSize(6,5);         // 指定标定板角点数
    std::vector<cv::Point2f> corners;
    bool mCornersFound;
    int flags =  CV_CALIB_CB_ADAPTIVE_THRESH +
                CV_CALIB_CB_NORMALIZE_IMAGE +
                CV_CALIB_CB_FILTER_QUADS +
                CV_CALIB_CB_FAST_CHECK;

    mCornersFound = cv::findChessboardCorners(rectifyImageL, boardSize, corners, flags);
    if(!mCornersFound)
    {
      ROS_ERROR("not able to detect chessboard..");
      return Eigen::Isometry3f::Identity();
    }

    float squareSize = 0.1;     // 米

    const cv::Point3f pointO(boardSize.height*0.1/2.0, boardSize.width*0.1/2.0, 0.0);
    for(int i = 0; i < boardSize.height; i++)
    {
        for(int j = 0; j < boardSize.width; j++)
        {
            cv::Point3f p;
            p.x = (i - pointO.x)*squareSize;
            p.y = (j - pointO.y)*squareSize;
            p.z = 0;
            bBoardCorner.push_back( p );
            //cout << p << endl;
        }
    }

    cout << "bBoardCorner size " << bBoardCorner.size() << endl;
    cout << "image corner size " << corners.size() << endl;

    cv::Mat rvec, tvec;
    cv::Mat inliers;
    cv::solvePnPRansac( bBoardCorner, corners, camK, cv::Mat(), rvec, tvec, true, 100, 4.0, 0.99, inliers );

    cout << "rvec:" << rvec << endl;
    cout << "tvec:" << tvec << endl;
    cout << inliers;


    Eigen::Vector3f v3 (rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
    double rad = v3.norm();
    v3.normalize();
    Eigen::AngleAxisf rotation_vector ( rad, v3 );

    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.rotate(rotation_vector.matrix());
    T.pretranslate(Eigen::Vector3f ( tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)) );

    return T;
}

myPointCloud::Ptr getPlane (myPointCloud::Ptr cloud, Eigen::Vector4d& line)
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

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

//  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  myPointCloud::Ptr final (new myPointCloud);

  pcl::copyPointCloud<pcl::PointXYZI> (*cloud, *inliers, *final);    /// 取出所有的内点

  line = Eigen::Vector4d(coefficients->values[0], coefficients->values[1],
                         coefficients->values[2], coefficients->values[3]);
  return final;
}


Eigen::Isometry3f txt2extrinsic(std::string filepath){

  Eigen::Matrix4f pose;
  std::ifstream infile(filepath, std::ios_base::binary);
  infile.read((char*)&pose, sizeof(Eigen::Matrix4f));
  infile.close();
  cout << pose <<endl;

  Eigen::Isometry3f T_lidar2board = Eigen::Isometry3f::Identity();
  T_lidar2board.rotate(pose.block<3,3>(0,0));
  T_lidar2board.pretranslate(pose.block<3,1>(0,3));

  return T_lidar2board;
}

bool calib_validator(Eigen::Isometry3f T_lidar2board) {

    std::vector<cv::Point3f> bBoardCorner;

    cv::Mat image = cv::imread("/home/ha/PCL_pcd/right5.jpg");
    Eigen::Isometry3f T_board2cam;
    T_board2cam = processImage(image, bBoardCorner);


    /// lidar to camera 外参
    Eigen::Vector3f v3 (-0.0203168, 0.00425956, -0.0473922);
    double rad = v3.norm();
    v3.normalize();
    Eigen::AngleAxisf rotation_vector ( rad, v3 );
    Eigen::Isometry3f T_lidar2cam22 = Eigen::Isometry3f::Identity();
    T_lidar2cam22.rotate(rotation_vector.matrix());
    T_lidar2cam22.pretranslate( Eigen::Vector3f( -0.239256, -0.172536, -0.101706 ) );

    cout << "T_lidar2board:\n" << T_lidar2board.matrix() << endl;
    cout << "T_lidar2cam22: \n" << T_lidar2cam22.matrix() << endl;


    Eigen::Quaternionf qlidarToCamera;
    qlidarToCamera = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())
                    *Eigen::AngleAxisf(-1.57, Eigen::Vector3f::UnitY())
                    *Eigen::AngleAxisf(1.57, Eigen::Vector3f::UnitX());
    Eigen::Isometry3f T_lidar2cam_axis = Eigen::Isometry3f::Identity();
    T_lidar2cam_axis.rotate(qlidarToCamera.matrix());


    Eigen::Isometry3f T_lidar2cam = Eigen::Isometry3f::Identity();
    T_lidar2cam = T_lidar2board * T_board2cam;
    cout << "T_lidar2cam: \n" << T_lidar2cam.matrix() << endl;
    T_lidar2cam = T_lidar2cam * T_lidar2cam_axis.inverse();




    std::vector<Eigen::Vector3f> boardCornerInlidar;
    for(int i = 0; i < bBoardCorner.size(); i++)
    {
      cv::Point3f pt = bBoardCorner.at(i);
      Eigen::Vector3f corner(pt.x, pt.y, pt.z);
      corner = T_lidar2cam22.inverse() * T_lidar2cam_axis.inverse() * T_board2cam * corner;

      boardCornerInlidar.push_back(corner);
    }



    cout << "readFromPCD begin ........\n";
    myPointCloud::Ptr cloud (new myPointCloud);
    std::string pcdfile = "/home/ha/PCL_pcd/boardCloud5.pcd";
    if ( pcl::io::loadPCDFile <myPoint> (pcdfile, *cloud) == -1)
    {
      std::cout << "Cloud reading failed --> " << pcdfile << std::endl;
      return (-1);
    }

    Eigen::Vector4d line;
    myPointCloud::Ptr final (new myPointCloud);
    final = getPlane(cloud, line);                    /// 取出平面

    double sum = 0;
    Eigen::Vector3f line_abc(line(0), line(1), line(2));
    for(int i = 0; i < boardCornerInlidar.size(); i++)
    {
        Eigen::Vector3f pt3f = boardCornerInlidar.at(i);

        double dis = (line_abc.dot(pt3f) + line(3) ) / line_abc.norm();
        cout << dis << endl;

        sum += dis;

    }

    cout << "error in m " << sum / boardCornerInlidar.size() << endl;
    cout << "error in m " << sum / double(boardCornerInlidar.size()) << endl;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_validator");
    ros::NodeHandle nh;


    std::string posefile = "/home/ha/PCL_pcd/T_lidar2board5.bin";

    Eigen::Isometry3f T_lidar2board = txt2extrinsic(posefile);

    calib_validator(T_lidar2board);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
      //ROS_INFO_STREAM("spinOnce");
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
