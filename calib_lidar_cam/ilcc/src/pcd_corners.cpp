#include "ros/ros.h"
#include <sstream>
#include <cmath>

#include <sensor_msgs/PointCloud2.h>           /// sensor_msgs::PointCloud2


#include <pcl/common/common.h>                 /// pcl::PointXYZINormal
#include <pcl_conversions/pcl_conversions.h>   /// fromROSMsg toROSMsg
#include <pcl/common/transforms.h>             /// transformPointCloud


#include "ilcc/utils.h"


#include <pcl/segmentation/sac_segmentation.h>


#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/calib3d/calib3d.hpp>

#include "extractCorner/cornersFromTXT.h"
#include "extractCorner/Optimization.h"
#include "extractCorner/Visualization.h"

ros::Publisher pubLaserCloud;
ros::Publisher pubLaserPCA;
ros::Publisher pubLaserOptim;
ros::Publisher pubLaserCorners;

Eigen::Vector2d gray_zone;
std::string save_path, corner_file_name, Optim_cloud_name;

std::string posefile;


Eigen::Vector2i board_size(7,8);
double grid_length = 0.15;
bool topleftWhite = false;

class CameraboardError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CameraboardError(const Eigen::Vector2i& board_size,
                       const bool topleftWhite,
                       const double grid_length,
                       const bool laser_white,
                       const bool useOutofBoard,
                       const Eigen::Vector2d& laserPoint,
                       const Eigen::Matrix2d& sqrtPrecisionMat):
          m_board_size(board_size),
          m_topleftWhite(topleftWhite),
          m_grid_length(grid_length),
          m_laser_white(laser_white),
          m_useOutofBoard(useOutofBoard),
          m_laserPoint(laserPoint),
          m_sqrtPrecisionMat(sqrtPrecisionMat) {}

    template <typename T>
    bool operator()(const T* const theta_t,
                    T* residuals) const
    {
        Eigen::Matrix<T,2,1> P = m_laserPoint.cast<T>();

        const T angle_axis[3] = {theta_t[0], T(0), T(0)};
        const T pt3[3] = {T(0), P(0), P(1)};
        T result[3];
        ceres::AngleAxisRotatePoint(angle_axis, pt3, result);
        result[1] = result[1] + theta_t[1];
        result[2] = result[2] + theta_t[2];


        T i = (result[1]+T(m_board_size(0))*T(m_grid_length)/T(2.0)) / T(m_grid_length);
        T j = (result[2]+T(m_board_size(1))*T(m_grid_length)/T(2.0)) / T(m_grid_length);
        // in board
        if( i > T(0) && i < T(m_board_size(0)) &&
            j > T(0) && j < T(m_board_size(1)) )
        {
            T ifloor = floor ( i );
            T jfloor = floor ( j );

            T ii = floor(ifloor / T(2)) *T(2);
            T jj = floor(jfloor / T(2)) *T(2);

            bool White = !m_topleftWhite;
            if(ifloor==ii && jfloor==jj)  // 都是偶数
                White = m_topleftWhite;
            if(ifloor!=ii && jfloor!=jj)  // 都是奇数
                White = m_topleftWhite;

            /// 颜色一致，无误差
            if(m_laser_white == White){
                residuals[0] = T(0);
            }
            else {
                T iceil = ceil ( i );
                T jceil = ceil ( j );
                T ierror, jerror;
                if(i-ifloor > T(0.5))
                    ierror = iceil - i;
                else
                    ierror = i-ifloor;

                if(j-jfloor > T(0.5))
                    jerror = jceil - j;
                else
                    jerror = j-jfloor;

                residuals[0] = ierror + jerror;
            }

        }
        // out of board
        else if(m_useOutofBoard){

            T ierror; /*= min( abs(i-T(0)) , abs(i - T(m_board_size(0))));*/
            T jerror; /*= min( abs(j-T(0)) , abs(j - T(m_board_size(1))));*/

            if( abs(i-T(0)) < abs(i - T(m_board_size(0))) )
              ierror = abs(i-T(0));
            else
              ierror = abs(i - T(m_board_size(0)));

            if( abs(j-T(0)) < abs(j - T(m_board_size(1))) )
              jerror = abs(j-T(0));
            else
              jerror = abs(j - T(m_board_size(1)));

            residuals[0] = ierror + jerror;
        } else {
          residuals[0] = T(0);
        }

        return true;
    }

private:
    // camera intrinsics

    double m_grid_length;
    Eigen::Vector2i m_board_size;  // width height
    bool m_topleftWhite;
    bool m_laser_white;

    bool m_useOutofBoard;

    Eigen::Vector2d m_laserPoint;

    // square root of precision matrix
    Eigen::Matrix2d m_sqrtPrecisionMat; // 误差项的权重
};



Eigen::Vector3d get_theta_t(myPointCloud::Ptr cloud,
                            bool useOutofBoard=true, Eigen::Vector3d theta_t=Eigen::Vector3d(0,0,0)) {

    Eigen::Vector3i black_gray_white(0,0,0);
    ceres::Problem problem;
//    Eigen::Vector3d theta_t(0,0,0);
    myPoint temp;
    for ( unsigned int i=0; i < cloud->size(); i++ )
    {
        temp = cloud->points[i];
        ceres::CostFunction* cost_function = 0;

        bool laser_white;
        if(temp.intensity < gray_zone[0]) {
            laser_white = false;
//            cloud->points[i].intensity = 50;
            black_gray_white(0)++;
        }
        else if(temp.intensity > gray_zone[1]) {
            laser_white = true;
//            cloud->points[i].intensity = 250;
            black_gray_white(2)++;
        }
        else {
//            cloud->points[i].intensity = 100;
            black_gray_white(1)++;
            continue;
        }

        Eigen::Vector2d laserPoint(temp.y, temp.z); // = point2d.at(i);
        Eigen::Matrix2d sqrtPrecisionMat = Eigen::Matrix2d::Identity();


        cost_function = new ceres::AutoDiffCostFunction<CameraboardError, 1, 3>(
                       new CameraboardError(board_size, topleftWhite, grid_length, laser_white,
                                            useOutofBoard, laserPoint, sqrtPrecisionMat));

        //new ceres::CauchyLoss(0.5)
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 theta_t.data());
    }

    cout << "black_gray_white: " << black_gray_white.transpose() << endl;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    cout << theta_t.transpose() << endl;
    return theta_t;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_pcd(myPointCloud::Ptr cloud) {

  int white = 0;
  int gray = 0;
  int black = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  myPoint temp;
  pcl::PointXYZRGB rgbtemp;
  for ( unsigned int i=0; i < cloud->size(); i++ )
  {
      temp = cloud->points[i];

      if(temp.intensity < gray_zone[0]) {
          temp.intensity = 50;
          rgbtemp.r = 0;
          rgbtemp.g = 0;
          rgbtemp.b = 0;
          black++;
      }
      else if(temp.intensity > gray_zone[1]) {
          temp.intensity = 255;
          rgbtemp.r = 255;
          rgbtemp.g = 255;
          rgbtemp.b = 255;
          white++;
      }
      else {
          temp.intensity = 100;
          rgbtemp.r = 255;
          rgbtemp.g = 0;
          rgbtemp.b = 0;
          gray++;
      }

      cloud->points[i].intensity = temp.intensity;

      rgbtemp.x = temp.x;
      rgbtemp.y = temp.y;
      rgbtemp.z = temp.z;
      rgbcloud->points.push_back(rgbtemp);
  }

  cloud->points[1].intensity = 0;

  cout << "rgbcloud size: " << rgbcloud->size() << endl;
  cout << "black: " << black << " gray: " << gray<< " white: " << white <<endl;
  return rgbcloud;
}


void extrinsic2txt(Eigen::Matrix4f transPCA, Eigen::Matrix4f transOptim){

  Eigen::Isometry3f T_optim = Eigen::Isometry3f::Identity();
  T_optim.rotate(transOptim.block<3,3>(0,0));
  T_optim.pretranslate(transOptim.block<3,1>(0,3));

  Eigen::Isometry3f T_PCA = Eigen::Isometry3f::Identity();
  T_PCA.rotate(transPCA.block<3,3>(0,0));
  T_PCA.pretranslate(transPCA.block<3,1>(0,3));

  Eigen::Isometry3f T_lidar2board = Eigen::Isometry3f::Identity();
  T_lidar2board = T_PCA.inverse() * T_optim.inverse();

  std::string filename = save_path + posefile;
  std::ofstream outfile(filename.c_str(), std::ios_base::binary);
  outfile.write((const char*)&T_lidar2board, sizeof(Eigen::Matrix4f));
  outfile.close();
}

myPointCloud getPCDcorners(Eigen::Matrix4f transPCA, Eigen::Matrix4f transOptim) {


  std::vector<double> x_grid_arr;
  std::vector<double> y_grid_arr;

  double temp;
  for(int i = 1; i < board_size(0); i++) {
      temp = (i-double(board_size(0))/2.0) * grid_length;
      x_grid_arr.push_back(temp);
  }
  for(int i = 1; i < board_size(1); i++) {
    temp = (i - double(board_size(1))/2.0) * grid_length;
    y_grid_arr.push_back(temp);
  }

  cout << x_grid_arr.size() << " " << y_grid_arr.size() << endl;

  myPointCloud pcd_corners;
  for(unsigned int i = 0; i < x_grid_arr.size(); i++)
    for(unsigned int j = 0; j < y_grid_arr.size(); j++) {

      myPoint pt;
      pt.x = 0;
      pt.y = x_grid_arr.at(i);
      pt.z = y_grid_arr.at(j);
      pt.intensity = 50;
      pcd_corners.push_back(pt);

    }


//  cout << "test \n";
//    cout << transOptim << endl;

//    Eigen::Matrix3f R = transOptim.block<3,3>(0,0);
//    cout << R << endl;

//    Eigen::Vector3f t = transOptim.block<3,1>(0,3);
//    cout << t << endl;

    Eigen::Isometry3f T_optim = Eigen::Isometry3f::Identity();
    T_optim.rotate(transOptim.block<3,3>(0,0));
    T_optim.pretranslate(transOptim.block<3,1>(0,3));

    Eigen::Isometry3f T_PCA = Eigen::Isometry3f::Identity();
    T_PCA.rotate(transPCA.block<3,3>(0,0));
    T_PCA.pretranslate(transPCA.block<3,1>(0,3));

    Eigen::Isometry3f T_lidar2board = Eigen::Isometry3f::Identity();
    T_lidar2board = T_PCA.inverse() * T_optim.inverse();

//    Eigen::Vector3f corner;
//    for(unsigned int i = 0; i < pcd_corners.size(); i++) {
//       corner = Eigen::Vector3f(pcd_corners.at(i).x, pcd_corners.at(i).y, pcd_corners.at(i).z);

////       corner = transPCA.transpose() * transOptim.transpose() * corner;
//       corner = T_lidar2board * corner;
//       cout << i << " " << corner.transpose() << endl;
//    }

    pcl::transformPointCloud(pcd_corners, pcd_corners, transOptim.inverse());
    pcl::transformPointCloud(pcd_corners, pcd_corners, transPCA.inverse());



    cout << "transOptim: " << endl;
    cout << transOptim << endl;
    cout << "transPCA: " << endl;
    cout << transPCA << endl;

    return pcd_corners;
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


myPointCloud transformbyPCA(myPointCloud::Ptr input_cloud) {

    /// PCA 降维度
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*input_cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*input_cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
    transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pcaCentroid.head<3>());

    pcl::PointCloud<myPoint>::Ptr PCA_cloud(new pcl::PointCloud<myPoint>);
    pcl::transformPointCloud(*input_cloud, *PCA_cloud, transform);

    cout << "PCA eigen values \n";
    std::cout << eigenValuesPCA << std::endl;
    std::cout << eigenVectorsPCA << std::endl;

    sensor_msgs::PointCloud2 PonitfiltedPCA;
    pcl::toROSMsg(*PCA_cloud, PonitfiltedPCA);
    PonitfiltedPCA.header.stamp = PonitfiltedPCA.header.stamp;
    PonitfiltedPCA.header.frame_id = "/velodyne";
    pubLaserPCA.publish(PonitfiltedPCA);


    /// 拟合虚拟棋盘格
    pcl::PointCloud<myPoint>::Ptr new_cloud(new pcl::PointCloud<myPoint>);

    Eigen::Vector3d theta_t;
    theta_t = get_theta_t(PCA_cloud);
    cout << "theta_t: " << theta_t.transpose() << endl;
    theta_t = get_theta_t(PCA_cloud, false, theta_t);
    cout << "theta_t: " << theta_t.transpose() << endl;

    Eigen::Affine3f transf = pcl::getTransformation(0, theta_t(1), theta_t(2),
                                                    theta_t(0), 0, 0);
    pcl::transformPointCloud(*PCA_cloud, *new_cloud, transf);

    sensor_msgs::PointCloud2 PonitfiltedOptim;
    pcl::toROSMsg(*new_cloud, PonitfiltedOptim);
    PonitfiltedOptim.header.stamp = PonitfiltedOptim.header.stamp;
    PonitfiltedOptim.header.frame_id = "/velodyne";
    pubLaserOptim.publish(PonitfiltedOptim);

    point2txt(new_cloud, save_path+Optim_cloud_name+".txt");

    cout << "optimization transform: \n";
    cout << transf.matrix() <<endl;

    /// color
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    new_cloud = color_pcd(transformedCloud);

    extrinsic2txt(transform, transf.matrix());

    myPointCloud pcd_corners;
    pcd_corners = getPCDcorners(transform, transf.matrix());

    sensor_msgs::PointCloud2 Ponitcorners;
    pcl::toROSMsg(pcd_corners, Ponitcorners);
    Ponitcorners.header.stamp = Ponitcorners.header.stamp;
    Ponitcorners.header.frame_id = "/velodyne";
    pubLaserCorners.publish(Ponitcorners);

    return pcd_corners;

}





void pcd_corner_est(myPointCloud::Ptr pointcloud) {


  pcl::PointCloud<myPoint> pcd_corners;

  pcd_corners = transformbyPCA(pointcloud);

  point2txt(pcd_corners.makeShared(), save_path+corner_file_name+".txt");
}


void callback_lidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar)
{

  myPointCloud pointcloud;

  pcl::fromROSMsg(*msg_lidar, pointcloud);

  pcd_corner_est(pointcloud.makeShared());
}




int readFromPCD(std::string pcdfile) {

  cout << "readFromPCD begin ........\n";
  myPointCloud::Ptr cloud (new myPointCloud);
  if ( pcl::io::loadPCDFile <myPoint> (pcdfile+".pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed --> " << pcdfile << std::endl;
    return (-1);
  }


  sensor_msgs::PointCloud2 Ponitboard;
  pcl::toROSMsg(*cloud, Ponitboard);
  Ponitboard.header.stamp = Ponitboard.header.stamp;
  Ponitboard.header.frame_id = "/velodyne";
  pubLaserCloud.publish(Ponitboard);



  pcd_corner_est(cloud);

  cout << "readFromPCD end ...........\n";
  return 0;
}


Eigen::Vector2d get_gray_zone(std::string lidar_path, double rate) {

  std::cout << "get_gray_zone\n";
  std::vector<Eigen::Vector3d> point3d;
  std::vector<double> intensitys;

  read_lidar_XYZI(point3d, intensitys, lidar_path);

  Eigen::Vector2d gray_zone, RLRH;
  Visualization visual;
  RLRH = visual.calHist(intensitys);

  gray_zone(0) = ((rate-1)*RLRH(0)+RLRH(1))/rate;
  gray_zone(1) = (RLRH(0)+(rate-1)*RLRH(1))/rate;

  cout << "gray_zone: " << gray_zone.transpose() << endl;

  return gray_zone;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_corners");
    ros::NodeHandle nh;

    float low_intensity, high_intensity, rate;

    std::string pcdfilepath;

    ros::NodeHandle nh_private("~");

    nh_private.param<float>("rate_intensity", rate, 4);
    nh_private.param<std::string>("pcd_file_path", pcdfilepath, "");
    nh_private.param<std::string>("save_path", save_path, "");
    nh_private.param<std::string>("corner_file_name", corner_file_name, "");
    nh_private.param<std::string>("Optim_cloud_name", Optim_cloud_name, "");

    nh_private.param<std::string>("posefile", posefile, "");


    gray_zone = get_gray_zone(pcdfilepath+".txt" , rate);
    cout << "rate " << rate << endl;


    cout << "gray_zone " <<  gray_zone.transpose() << endl;

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/filtered", 10);
    pubLaserPCA = nh.advertise<sensor_msgs::PointCloud2>("/PCA_cloud", 10);
    pubLaserOptim = nh.advertise<sensor_msgs::PointCloud2>("/Optim_cloud", 10);
    pubLaserCorners = nh.advertise<sensor_msgs::PointCloud2>("/pcd_corners", 10);


//    ros::Subscriber sub_lidar = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1,
//                                                                       callback_lidar);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
      readFromPCD(pcdfilepath);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
