#ifndef _CORNERFROMTXT_H
#define _CORNERFROMTXT_H

#include <iostream>
#include <opencv2/core.hpp>
#include <Eigen/Core>


void readCorners(std::string Xpath,
                 std::string Ypath,
                 std::vector<std::vector<cv::Point2f> >& Corners);


void read_corners(std::vector<cv::Point3d>& point3d,
                  std::vector<cv::Point2d>& point2d,
                  std::string lidar_path,
                  std::string cam_path);

void read_lidar_XYZI(std::vector<Eigen::Vector3d>& point3d,
                  std::vector<double>& intensitys,
                  std::string lidar_path);

void pointFromtxt(std::vector<Eigen::Vector2d>& point2d,
                  std::vector<int>& intensitys,
                  std::string filepath);

#endif
