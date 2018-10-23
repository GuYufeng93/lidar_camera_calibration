#include "Optimization.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>  /// cv::solvePnPRansac


Optimization::Optimization()
{

}


Eigen::Isometry3d Optimization::solvePose3d2dError(std::vector<cv::Point3d> pts3d,
                                     std::vector<cv::Point2d> pts2d,
                                     cv::Mat K)

{

    Eigen::Quaterniond qlidarToCamera;              /// 这个是为了让相机和激光坐标系方向一致
    qlidarToCamera = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
                    *Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitY())
                    *Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());

    Eigen::Isometry3d Trans = Eigen::Isometry3d::Identity();
    Trans.rotate ( qlidarToCamera.matrix() );
    Trans.pretranslate ( Eigen::Vector3d ( 0,0,0 ) );


    Eigen::Vector4d camera;
    camera(0) = K.at<double>(0,0);
    camera(1) = K.at<double>(0,2);
    camera(2) = K.at<double>(1,1);
    camera(3) = K.at<double>(1,2);

    Eigen::Vector3d r_ceres(0,0,0);
    Eigen::Vector3d t_ceres(0,0,0);

//    cv::Mat rvec, tvec;
//    cv::solvePnPRansac( pts3d, pts2d, K, cv::Mat(), rvec, tvec, true, 100, 4.0, 0.99);
//    for(int i = 0; i < 3; i++)
//    {
//        r_ceres(i) = rvec.at<double>(i);
//        t_ceres(i) = tvec.at<double>(i);
//    }

    std::cout << "r_initial: " << r_ceres.transpose() << std::endl;
    std::cout << "t_initial: " << t_ceres.transpose() << std::endl;


    //优化
    ceres::Problem problem;
    for ( unsigned int i=0; i<pts3d.size(); i++ )
    {
        Eigen::Vector3d v ( pts3d[i].x, pts3d[i].y, pts3d[i].z );
        Eigen::Vector3d v_transformed = Trans * v;
        cv::Point3d pt(v_transformed(0), v_transformed(1), v_transformed(2));

        ceres::CostFunction* cost_function = 0;

        cost_function = new ceres::AutoDiffCostFunction<Pose3d2dError, 2, 3, 3>(
                       new Pose3d2dError(pt, pts2d[i], camera));

        problem.AddResidualBlock(cost_function,
                                 new ceres::CauchyLoss(0.5),
                                 r_ceres.data(),
                                 t_ceres.data());
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    //options.max_num_iterations = 20;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    std::cout << "r_ceres: " << r_ceres.transpose() << std::endl;
    std::cout << "t_ceres: " << t_ceres.transpose() << std::endl;


    double rad = r_ceres.norm();
    r_ceres.normalize();
    Eigen::AngleAxisd rotation_vector ( rad, r_ceres );     //用旋转向量构造旋转向量！


    Eigen::Vector3d euler_angles = rotation_vector.matrix().eulerAngles ( 2,1,0 ); // ZYX顺序，即roll pitch yaw顺序
    std::cout<<"yaw pitch roll = "<<(euler_angles*180/3.14).transpose()<<std::endl;


    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector.matrix());
    T.pretranslate(Eigen::Vector3d ( t_ceres[0], t_ceres[1], t_ceres[2]));

//    cout << "Eigen::Isometry3d \n" << T.matrix() << endl;
    return T;
}



Eigen::Vector3d Optimization::get_theta_t(std::vector<Eigen::Vector2d> point2d, std::vector<int> intensitys)
{

    Eigen::Vector2d gray_zone_temp(1.76, 48.88);
    Eigen::Vector2d gray_zone;;
    double rate = 4.0;
    gray_zone(0) = ((rate-1)*gray_zone_temp(0)+gray_zone_temp(1))/rate;
    gray_zone(1) = (gray_zone_temp(0)+(rate-1)*gray_zone_temp(1))/rate;
    std::cout << "gray_zone " <<  gray_zone.transpose() << std::endl;


    Eigen::Vector2i board_size(7,8);
    bool topleftWhite = true;
    double grid_length = 0.1;

    ceres::Problem problem;
    Eigen::Vector3d theta_t(0,0,0);
    for ( unsigned int i=0; i<point2d.size(); i++ )
    {
        ceres::CostFunction* cost_function = 0;

        bool laser_white;
        if(intensitys.at(i) < gray_zone[0])
            laser_white = false;
        else if(intensitys.at(i) > gray_zone[1])
            laser_white = true;
        else
            continue;

        Eigen::Vector2d laserPoint = point2d.at(i);
        Eigen::Matrix2d sqrtPrecisionMat = Eigen::Matrix2d::Identity();


        cost_function = new ceres::AutoDiffCostFunction<VirtualboardError, 1, 3>(
                       new VirtualboardError(board_size, topleftWhite, grid_length, laser_white,
                                            laserPoint, sqrtPrecisionMat));

        //new ceres::CauchyLoss(0.5)
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 theta_t.data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";


    std::cout << theta_t.transpose() << std::endl;
    return theta_t;
}



