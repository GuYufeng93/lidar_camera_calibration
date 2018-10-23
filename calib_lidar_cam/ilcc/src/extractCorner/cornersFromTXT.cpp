#include "cornersFromTXT.h"

#include <fstream>
#include <Eigen/Core>

using namespace std;


void split(std::string& s, std::string& delim, std::vector<std::string>& ret)
{
    size_t last = 0;
    size_t index = s.find_first_of(delim, last);
    while (index != string::npos)
    {
        ret.push_back(s.substr(last, index - last));
        last = index + 1;
        index = s.find_first_of(delim, last);
    }
    if (index - last > 0)
    {
        ret.push_back(s.substr(last, index - last));
    }
}

//void readCorners(std::string Xpath,
//                 std::string Ypath,
//                 std::vector<std::vector<cv::Point2f> >& Corners)
//{
//    std::ifstream ifsX, ifsY;

//    ifsX.open(Xpath);
//    ifsY.open(Ypath);

//    std::string s;
//    int n = 0;
//    std::string dlim = " ";
//    while ( getline(ifsX, s))
//    {
//        vector<string> slist;
//        split(s, dlim, slist);
//        vector<cv::Point2f> corner;
//        for (uint i = 0; i < slist.size(); i++)
//        {
//            stringstream ss;
//            ss << slist[i];
//            cv::Point2f p;
//            ss >> p.x;
//            p.y = 0;
//            corner.push_back(p);
//        }
//        Corners.push_back(corner);
//    }
//    while ( getline(ifsY, s))
//    {
//        vector<string> slist;
//        split(s, dlim, slist);
//        for (uint i = 0; i < slist.size(); i++)
//        {
//            stringstream ss;
//            ss << slist[i];
//            ss >> Corners[n][i].y;
//        }
//        n++;
//    }

//    ifsX.close();
//    ifsY.close();
//}

void read_corners(std::vector<cv::Point3d>& point3d,
                  std::vector<cv::Point2d>& point2d,
                  string lidar_path,
                  string cam_path) {

    std::ifstream infile;
    string num;

    int counter = 0;
    infile.open(lidar_path.c_str());
    infile>>num;
    while ( !infile.eof() )
    {
        float x, y, z, inten;
        infile>>x>>y>>z>>inten;
        point3d.push_back( cv::Point3d(x,y,z));
        counter++;

        //cout << counter << " " << x << " " << y << endl;
    }
    infile.close();


    counter = 0;
    infile.open(cam_path.c_str());
    infile>>num;
    while ( !infile.eof() )
    {
        float x, y;
        infile>>x>>y;
        point2d.push_back( cv::Point2d(x,y));

        counter++;
        //cout << counter << " " << x << " " << y << endl;

    }
    infile.close();

    point2d.pop_back();
    point3d.pop_back();

    cout << " lidar  size: " << point3d.size() << endl;
    cout << " camera size: " << point2d.size() << endl;
}

void read_lidar_XYZI(std::vector<Eigen::Vector3d>& point3d,
                  std::vector<double>& intensitys,
                     std::string lidar_path){

  std::ifstream infile;
  string num;

  int counter = 0;
  infile.open(lidar_path.c_str());
  infile>>num;
  while ( !infile.eof() )
  {
      float x, y, z, inten;
      infile>>x>>y>>z>>inten;
      point3d.push_back( Eigen::Vector3d(x,y,z));
      intensitys.push_back(inten);
      counter++;

      //cout << counter << " " << x << " " << y << endl;
  }
  infile.close();

  intensitys.pop_back();
  point3d.pop_back();

  std::cout << "lidar point size: " << point3d.size() << endl;
}

void pointFromtxt(std::vector<Eigen::Vector2d>& point2d,
                  std::vector<int>& intensitys,
                  string filepath)
{
    std::ifstream infile(filepath.c_str(), std::ios_base::in);
    string num;
    infile>>num;
    while ( !infile.eof() )
    {
        double x, y, z;
        int intensity;
        infile>>x>>y>>z>>intensity;
        point2d.push_back( Eigen::Vector2d(y,z));
        intensitys.push_back(intensity);
    }
    infile.close();
    cout << "point2d size: " << point2d.size() << endl;

}

