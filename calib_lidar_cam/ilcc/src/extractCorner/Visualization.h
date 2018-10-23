#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <iostream>
#include <eigen3/Eigen/Core>

#include <opencv2/core.hpp>


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


        if ( P_c[2]<0 || P_c[2] > 5) {
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


class Visualization
{
public:
    Visualization();

    cv::Mat getChessboard(double block_pixel = 100, double edgescale = 0.1);
    Eigen::Vector2d calHist(std::vector<double> datas);

};



#endif // VISUALIZATION_H
