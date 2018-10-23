#include "ros/ros.h"
#include <sstream>
#include <cmath>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Image.h>       /// sensor_msgs::Image

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "ilcc/utils.h"

std::string image_topic, save_path, save_image_name, corner_file_name;
bool save_image_flag;

bool processImage(cv::Mat image) {

  cv::Size boardSize(6,7);         // 指定标定板角点数
  std::vector<cv::Point2f> corners;
  bool mCornersFound;
  int flags =  CV_CALIB_CB_ADAPTIVE_THRESH +
              CV_CALIB_CB_NORMALIZE_IMAGE +
              CV_CALIB_CB_FILTER_QUADS +
              CV_CALIB_CB_FAST_CHECK;

  cv::Size imageSize = cv::Size(image.cols, image.rows);
  cv::Mat M1l, M2l;     // 映射表
  cv::Mat R_l;

  cv::Mat camK, distort_param;

  cv::FileStorage fs;
  fs.open("/home/ha/PCL_pcd/stereo.yaml", cv::FileStorage::READ);
  fs["Kl"] >> camK;
  fs["dl"] >> distort_param;
  fs.release();

  cv::initUndistortRectifyMap(camK, distort_param, R_l, camK, imageSize, CV_32F, M1l, M2l);

  cv::Mat rectifyImageL;
  cv::remap(image, rectifyImageL, M1l, M2l, cv::INTER_LINEAR);


  mCornersFound = cv::findChessboardCorners(rectifyImageL, boardSize, corners,flags);
  if(!mCornersFound)
  {
    ROS_ERROR("not able to detect chessboard..");
    return false;
  }


  if(save_image_flag)
  {
    std::string filename = save_path+corner_file_name;
    std::ofstream outfile(filename.c_str(), std::ios_base::trunc);
    outfile << "#"<<corners.size() << endl;
    for(unsigned int i = 0; i < corners.size(); i++)
        outfile << corners.at(i).x << " " << corners.at(i).y << endl;
    outfile.close();
  }

  cv::drawChessboardCorners(rectifyImageL, boardSize, corners, true);
  cv::resize(rectifyImageL, rectifyImageL, cv::Size(rectifyImageL.cols/1.5, rectifyImageL.rows/1.5));
  cv::imshow("chessboard", rectifyImageL);
  cv::waitKey(5);

  ROS_INFO("image corner save to %s%s", save_path.c_str(), corner_file_name.c_str());
  return true;

}



void callback_image(const sensor_msgs::ImageConstPtr& msg_img)
{

  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  }

  // sensor_msgs::Image to OpenCV Mat structure
  cv::Mat Image = cv_ptr->image;

  processImage(Image);

  if(save_image_flag)
    cv::imwrite(save_path+save_image_name, Image);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_corners");
    ros::NodeHandle nh;




    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("image_topic", image_topic, "/ged231/image/frontright");
    nh_private.param<std::string>("save_path", save_path, "/home/ha/PCL_pcd/");
    nh_private.param<std::string>("save_image_name", save_image_name, "image.jpg");
    nh_private.param<std::string>("corner_file_name", corner_file_name, "image.txt");
    nh_private.param<bool>("save_image_flag", save_image_flag, "false");

    ros::Subscriber subImage = nh.subscribe<sensor_msgs::Image> (image_topic, 5, callback_image);


    ros::Rate loop_rate(1);
    while (ros::ok())
    {
      //ROS_INFO_STREAM("spinOnce");
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
