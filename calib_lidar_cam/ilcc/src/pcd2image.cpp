#include <cstdlib>
#include <cstdio>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>


using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;



class PinholeCamera
{

public:

   PinholeCamera(){}

   void setCam( cv::Mat K , cv::Mat distortParam, double width, double height) {

        m_fx = K.at<double>(0,0);
        m_cx = K.at<double>(0,2);
        m_fy = K.at<double>(1,1);
        m_cy = K.at<double>(1,2);

        m_k1 = distortParam.at<double>(0,0);
        m_k2 = distortParam.at<double>(0,1);
        m_p1 = distortParam.at<double>(0,2);
        m_p2 = distortParam.at<double>(0,3);

        m_imageWidth = width;
        m_imageHeight = height;
    }

    void setRt(Eigen::Matrix3d R, Eigen::Vector3d t) {
      m_R = R;
      m_t = t;
    }

    bool spaceToPlane( Eigen::Vector3d P_w, Eigen::Vector2d &P_cam)
    {
        Eigen::Vector3d P_c = m_R * P_w + m_t;
//      Eigen::Vector3d P_c = m_R.transpose() * (P_w - m_t);


        if ( P_c[2]<0 || P_c[2] > 3) {
//            cout << P_c.transpose() << endl;
            return false;
        }

        // Transform to model plane
        double u = P_c[0] / P_c[2];
        double v = P_c[1] / P_c[2];

        double rho_sqr = u * u + v * v;

        double L = 1.0 + m_k1 * rho_sqr + m_k2 * rho_sqr * rho_sqr;
        double du = 2.0 * m_p1 * u * v + m_p2 * ( rho_sqr + 2.0 * u * u );
        double dv = m_p1 * ( rho_sqr + 2.0 * v * v ) + 2.0  * m_p2 * u * v;

        u      = L * u + du;
        v      = L * v + dv;

        P_cam(0) = m_fx * u + m_cx;
        P_cam(1) = m_fy * v + m_cy;

//        cout << "P_cam: " << P_cam.transpose() << endl;

        if ( P_cam(0)>0 && P_cam(0)<m_imageWidth && P_cam(1)>0 && P_cam(1)<m_imageHeight)
          return true;
        else
          return false;
    }

    Eigen::Matrix3d m_R;
    Eigen::Vector3d m_t;

    int m_imageWidth;
    int m_imageHeight;

    double m_fx;
    double m_fy;
    double m_cx;
    double m_cy;
    double m_k1;
    double m_k2;
    double m_p1;
    double m_p2;
};



PinholeCamera pinholecam;


void HSVtoRGB(unsigned char *r, unsigned char *g, unsigned char *b, int h, int s, int v)
{
  // convert from HSV/HSB to RGB color
  // R,G,B from 0-255, H from 0-260, S,V from 0-100
  // ref http://colorizer.org/

  // The hue (H) of a color refers to which pure color it resembles
  // The saturation (S) of a color describes how white the color is
  // The value (V) of a color, also called its lightness, describes how dark the color is

  int i;


  float RGB_min, RGB_max;
  RGB_max = v*2.55f;
  RGB_min = RGB_max*(100 - s)/ 100.0f;

  i = h / 60;
  int difs = h % 60; // factorial part of h

  // RGB adjustment amount by hue
  float RGB_Adj = (RGB_max - RGB_min)*difs / 60.0f;

  switch (i) {
  case 0:
    *r = RGB_max;
    *g = RGB_min + RGB_Adj;
    *b = RGB_min;
    break;
  case 1:
    *r = RGB_max - RGB_Adj;
    *g = RGB_max;
    *b = RGB_min;
    break;
  case 2:
    *r = RGB_min;
    *g = RGB_max;
    *b = RGB_min + RGB_Adj;
    break;
  case 3:
    *r = RGB_min;
    *g = RGB_max - RGB_Adj;
    *b = RGB_max;
    break;
  case 4:
    *r = RGB_min + RGB_Adj;
    *g = RGB_min;
    *b = RGB_max;
    break;
  default:		// case 5:
    *r = RGB_max;
    *g = RGB_min;
    *b = RGB_max - RGB_Adj;
    break;
  }
}

void processData(cv::Mat image, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){

  pcl::PointXYZI ptmp;

  /// 这个是为了让相机和激光坐标系方向一致
  Eigen::Affine3f transf = pcl::getTransformation(0, 0, 0, 1.57, -1.57, 0);
  pcl::transformPointCloud(*cloud, *cloud, transf);

  cout<<"start project "<< cloud->size() << endl;
  int counter = 0;
  for(unsigned int index=0; index<cloud->size(); index++){

    ptmp = cloud->points[index];
    Eigen::Vector3d Pw(ptmp.x, ptmp.y, ptmp.z);
    Eigen::Vector2d Pcam;
    if ( pinholecam.spaceToPlane(Pw, Pcam) ) {

      int x = Pcam[0];
      int y = Pcam[1];

      unsigned char r = 255-ptmp.intensity;
      unsigned char g = 255-ptmp.intensity;
      unsigned char b = 255-ptmp.intensity;

      HSVtoRGB(&r, &g, &b, 255-ptmp.intensity*2, 70+ptmp.intensity, 50+ptmp.intensity);

      cv::circle(image, cv::Point2d(x,y), 1.2, cv::Scalar(r,g,b), 2);

//      image.ptr<uchar>(y)[x*3] = 255;
//      image.ptr<uchar>(y)[x*3+1] = 0;
//      image.ptr<uchar>(y)[x*3+2] = 0;

      counter++;
    }
  }
  cout << counter << " points ok\n";
  cv::resize(image, image, cv::Size(image.cols/1.5, image.rows/1.5));
  cv::imshow("img_liar_point", image);
  cv::waitKey(5);
}


void callback_LidarCam(const sensor_msgs::PointCloud2ConstPtr& msg_pc,
          const sensor_msgs::ImageConstPtr& msg_img)
{
  ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
  ROS_INFO_STREAM("image received at " << msg_img->header.stamp.toSec());

  pcl::PointCloud<pcl::PointXYZI> input_cloud;


  // Loading Velodyne point cloud_sub
  pcl::fromROSMsg(*msg_pc, input_cloud);

//  pcl::transformPointCloud(input_cloud, input_cloud, transOptim.inverse());
//  pcl::transformPointCloud(input_cloud, input_cloud, transPCA.inverse());


  cv::Mat img_left = cv_bridge::toCvCopy(msg_img,"bgr8")->image;

  processData(img_left, input_cloud.makeShared());
}


int main(int argc, char** argv){

  ros::init(argc,argv,"pcd2image");

  ros::NodeHandle n;


  cv::Mat camK, distort_param;

  cv::FileStorage fs;
  fs.open("/home/ha/PCL_pcd/monostereo.yaml", cv::FileStorage::READ);

  fs["Kr"] >> camK;
  fs["dr"] >> distort_param;
  fs.release();

  /// lidar to camera 外参
//  Eigen::Vector3d v3 (-0.015408, 0.00973733, 0.00349175);
//  Eigen::Vector3d v3 (-0.00581376,  0.00724243,   0.0231476);
  Eigen::Vector3d v3 ( -0.0100437, 0.000681339, -0.00713621);
  double rad = v3.norm();
  v3.normalize();
  Eigen::AngleAxisd rotation_vector ( rad, v3 );

  Eigen::Matrix3d R = rotation_vector.matrix();
//  Eigen::Vector3d t( 0.189213,  -0.210704, -0.0879011 );
//  Eigen::Vector3d t( 0.206319,  -0.183469, -0.0932258 );
  Eigen::Vector3d t( -0.236765,  -0.183143, -0.0850668 );
  cv::Mat image = cv::imread("/home/ha/PCL_pcd/right5.jpg");

  pinholecam.setRt( R, t );
  pinholecam.setCam(camK, distort_param, image.cols, image.rows);

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/velodyne_points", 2);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/ged231/image/frontright", 5);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), cloud_sub, image_sub);
  sync.registerCallback(boost::bind(&callback_LidarCam, _1, _2));

  while (ros::ok())
  {
    ros::spin();
  }
}



