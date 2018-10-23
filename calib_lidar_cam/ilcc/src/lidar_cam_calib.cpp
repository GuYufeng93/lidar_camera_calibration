#include <iostream>
#include <cmath>

#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "extractCorner/cornersFromTXT.h"
#include "extractCorner/Optimization.h"
#include "extractCorner/Visualization.h"

#include <ros/ros.h>

using namespace std;



void show_pcd_corners(std::string lidar_path, std::string cam_path) {

    std::vector<cv::Point3d> point3d;
    std::vector<cv::Point2d> point2d;

    read_corners(point3d, point2d, lidar_path, cam_path);
    reverse(point3d.begin(), point3d.end());

    cv::Mat img_show(1200, 1500, CV_8UC3);
    for (uint i = 0; i < point3d.size(); i++)
    {

        cv::Point2f p;
        p.x = -point3d.at(i).y;
        p.y = -point3d.at(i).z;

        p.x = (p.x + 0.7)*1000;
        p.y = (p.y + 0.2)*1000;
        cv::circle(img_show, p, 2, cv::Scalar(255,0,0), 3);

        stringstream ss;
        ss<<i;
        string str;
        ss>>str;
//        string str = to_string(i);
        cv::putText(img_show, str, p, 0.2, 0.4, cv::Scalar(0,0,255),1);

        cv::circle(img_show, point2d.at(i), 2, cv::Scalar(0,0,255), 3);
        cv::putText(img_show, str, point2d.at(i), 0.2, 0.4, cv::Scalar(0,0,255),1);

    }

    cv::imshow("pcd corners", img_show);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void extrinsic2txt(string savefile, Eigen::Matrix4d lidar2cam)
{
  std::ofstream outfile(savefile.c_str(), std::ios_base::binary);
  outfile.write((const char*)&lidar2cam, sizeof(Eigen::Matrix4d));
  outfile.close();
}


void lidar_cam_calib(std::string lidar_path,
                     std::string cam_path,
                     std::string image_path,
                     std::string yaml_path,
                     std::string extrinsic_save_file) {

    std::vector<cv::Point3d> point3d;
    std::vector<cv::Point2d> point2d;

    read_corners(point3d, point2d, lidar_path, cam_path);
    reverse(point3d.begin(), point3d.end());

    cv::Mat camK, distort_param;
    int imageWidth, imageHeight;

    cv::FileStorage fs;
    fs.open(yaml_path, cv::FileStorage::READ);
    fs["Kr"] >> camK;
    fs["dr"] >> distort_param;
    imageWidth = static_cast<int>(fs["Camera.width"]);
    imageHeight = static_cast<int>(fs["Camera.height"]);
    fs.release();

    cout << point3d.size() << " " << point2d.size() <<endl;
    Optimization optim;
    Eigen::Isometry3d T = optim.solvePose3d2dError(point3d,point2d,camK);

    extrinsic2txt(extrinsic_save_file, T.matrix());

#if 1
    //// 激光点投影回图像

    Eigen::Matrix3d R = T.rotation();
    Eigen::Vector3d t = T.translation();

    cv::Mat image = cv::imread(image_path);

    cout << "channels: " << image.channels() << endl;
    cout << image.cols << " " << image.rows << endl;

    PinholeCamera pinholecam;
    pinholecam.setRt( R, t );
    pinholecam.setCam(camK, distort_param, imageWidth, imageHeight);


    Eigen::Quaterniond qlidarToCamera;                      /// 这个是为了让相机和激光坐标系方向一致
    qlidarToCamera = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
                    *Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitY())
                    *Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());

    Eigen::Isometry3d Trans=Eigen::Isometry3d::Identity();
    Trans.rotate ( qlidarToCamera.matrix() );
    Trans.pretranslate ( Eigen::Vector3d ( 0,0,0 ) );

    cout<<"start project "<< point3d.size() << endl;


    float errorSum = 0.0f;
    float errorMax = std::numeric_limits<float>::min();

    for(unsigned int i=0; i<point3d.size(); i++)
    {
        Eigen::Vector3d v ( point3d.at(i).x, point3d.at(i).y, point3d.at(i).z );
        Eigen::Vector3d Pw = Trans * v;
        Eigen::Vector2d Pcam;
        if ( pinholecam.spaceToPlane(Pw, Pcam) ) {

            cv::Point2d pEst(Pcam[0], Pcam[1]);
            cv::Point2d pObs = point2d.at(i);

            cv::circle(image, pEst, 3, cv::Scalar(0,0,255), 3);
            cv::circle(image, pObs, 3, cv::Scalar(255,0,0), 3);

            float error = cv::norm(pObs - pEst);

            errorSum += error;
            if (error > errorMax)
            {
                errorMax = error;
            }
        }
    }

    std::ostringstream oss;
    oss << "Reprojection error: avg = " << errorSum / point2d.size()
        << "   max = " << errorMax;

    cv::putText(image, oss.str(), cv::Point(10, image.rows - 20),
                cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(255, 255, 255),
                1, CV_AA);


    cv::resize(image, image, cv::Size(image.cols/1.5, image.rows/1.5));
    cv::imshow("board", image);
    cv::waitKey(0);

#endif

    cv::destroyAllWindows();
}



void lidar_cam_calib_drectly(){

}
int main(int argc, char** argv){

  ros::init(argc,argv,"lidar_cam_calib");

  ros::NodeHandle nh;

  std::string lidar_corner_path, cam_corner_path;
  std::string image_path;
  std::string yaml_path;
  std::string extrinsic_save_file;
  bool show_corner = true;

  ros::NodeHandle nh_private("~");

  nh_private.param<std::string>("lidar_corner_path", lidar_corner_path, "/home/ha/PCL_pcd/boardCorner1.txt");
  nh_private.param<std::string>("cam_corner_path", cam_corner_path, "/home/ha/PCL_pcd/imageCorner1.txt");
  nh_private.param<std::string>("image_path", image_path, "/home/ha/PCL_pcd/right1.jpg");
  nh_private.param<std::string>("yaml_path", yaml_path, "/home/ha/PCL_pcd/monostereo.yaml");
  nh_private.param<std::string>("extrinsic_save_file", extrinsic_save_file, "/home/ha/PCL_pcd/T_lidar2cam1.bin");
  nh_private.param<bool>("show_corner", show_corner, "true");

  if(show_corner)
    show_pcd_corners(lidar_corner_path, cam_corner_path);
  else
    lidar_cam_calib(lidar_corner_path, cam_corner_path, image_path, yaml_path, extrinsic_save_file);

//  while (ros::ok())
//  {
//    ros::spin();
//  }

}


