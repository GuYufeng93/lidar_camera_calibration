
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camodocal/chessboard/Chessboard.h"

#include <opencv2/calib3d.hpp>

#include <iomanip>
#include <fstream>


#include "ros/ros.h"
#include <sstream>
#include <cmath>
#include <sensor_msgs/Image.h>       /// sensor_msgs::Image
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


using namespace std;


cv::Mat M1l, M2l;     // 映射表
bool save_image_flag = false;
std::string image_topic, save_path, save_image_name, corner_file_name;


void corner2txt(std::vector<cv::Point2f> corners, std::string filename) {

  std::ofstream outfile(filename.c_str(), std::ios_base::trunc);

  outfile << "#"<<corners.size() << endl;

  for(int i = 0; i < corners.size(); i++)
      outfile << corners.at(i).x << " " << corners.at(i).y << endl;

  outfile.close();
}

void getRectifyParam(std::string yaml_path) {

  cv::Mat camK, distort_param;
  int imageWidth, imageHeight;
  cv::FileStorage fs;
  fs.open(yaml_path, cv::FileStorage::READ);
  fs["Kl"] >> camK;
  fs["dl"] >> distort_param;
  imageWidth = static_cast<int>(fs["Camera.width"]);
  imageHeight = static_cast<int>(fs["Camera.height"]);
  fs.release();

  cout << "camK :"  << camK << endl;
  cout << "dist :" << distort_param << endl;

  cv::Mat R_l;
  cv::initUndistortRectifyMap(camK, distort_param, R_l, camK,  cv::Size(imageWidth,imageHeight),
                              CV_32F, M1l, M2l);
}


void processImage(cv::Mat image) {
  //Number of inner corners on the chessboard pattern
  cv::Size boardSize(7,6);         // 指定标定板角点数

  bool useOpenCV = false;



  cv::Mat rectifyImageL;
  cv::remap(image, rectifyImageL, M1l, M2l, cv::INTER_LINEAR);


  camodocal::Chessboard chessboard(boardSize, rectifyImageL);

  chessboard.findCorners(useOpenCV);
  if (chessboard.cornersFound())
      std::cerr << "# INFO: Detected chessboard in image " << std::endl;
  else
      std::cout << "can't Detected chessboard\n";

  if(save_image_flag) {
    string image_corner_path = "/home/ha/PCL_pcd/left1.txt";
    std::vector<cv::Point2f> corners = chessboard.getCorners();
    corner2txt(corners, image_corner_path);
  }


  cv::drawChessboardCorners(rectifyImageL, boardSize, chessboard.getCorners(), chessboard.cornersFound());
  cv::resize(rectifyImageL, rectifyImageL, cv::Size(rectifyImageL.cols/1.5, rectifyImageL.rows/1.5));
  cv::imshow("Corners", rectifyImageL);
  cv::waitKey(3);
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
    ros::init(argc, argv, "getBoardCorners");
    ros::NodeHandle nh;

    std::string yaml_path;

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("image_topic", image_topic, "/ged231/image/frontright");
    nh_private.param<std::string>("save_path", save_path, "/home/ha/PCL_pcd/");
    nh_private.param<std::string>("save_image_name", save_image_name, "image.jpg");
    nh_private.param<std::string>("corner_file_name", corner_file_name, "image.txt");
    nh_private.param<bool>("save_image_flag", save_image_flag, "false");

    nh_private.param<std::string>("yaml_path", yaml_path, "/home/ha/PCL_pcd/monostereo.yaml");

    getRectifyParam(yaml_path);


    ros::Subscriber subImage = nh.subscribe<sensor_msgs::Image> (image_topic, 5, callback_image);


    ros::spin();

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
      //ROS_INFO_STREAM("spinOnce");
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}

