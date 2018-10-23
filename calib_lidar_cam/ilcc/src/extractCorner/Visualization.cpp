#include "Visualization.h"

#include <iostream>
#include <map>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>



Visualization::Visualization()
{

}

cv::Mat Visualization::getChessboard(double block_pixel, double edgescale){

    //自定义标定板
    //double block_pixel = 100;

    double edge = block_pixel * edgescale;

    cv::Size imagesize(block_pixel * 10, block_pixel * 10);   // (x,y)

    unsigned char white = 255;
    unsigned char color_row = white;    // 第一个格子的颜色
    unsigned char color_col = white;


    cv::Mat chessBoard(imagesize.height+edge*2, imagesize.width+edge*2, CV_8UC3, cv::Scalar::all(white));


    for (int i = edge; i <= imagesize.width; i = i + block_pixel){
        for (int j = edge; j <= imagesize.height; j = j + block_pixel){
            cv::Mat ROI = chessBoard(cv::Rect(i, j, block_pixel, block_pixel));
            ROI.setTo(cv::Scalar::all(color_col));
            color_col = ~color_col;
        }
        color_row = ~color_row;
        color_col = color_row;
    }

    std::cout << chessBoard.channels() << std::endl;
//    cv::imshow("Chess board", chessBoard);

//    cvWaitKey(0);
    return chessBoard;
}

Eigen::Vector2d Visualization::calHist(std::vector<double> datas) {

    int HISTO_LENGTH = 100;
    std::vector<int> dataHist;  //反射强度直方图统计
    dataHist.reserve(HISTO_LENGTH);
    for(int i = 0; i < HISTO_LENGTH; i++)
        dataHist.push_back(0);

    sort(datas.begin(), datas.end());
    double min = datas.front();
    double max = datas.back();
    const double factor = HISTO_LENGTH/(max-min);

    for(unsigned int i = 0; i < datas.size(); i++) {
        double sample = datas.at(i) - min;
        int bin = round(sample*factor);    // 将sample分配到bin组
        dataHist[bin]++;
    }



    double sum = 0.0;
    for(unsigned int i = 0; i < datas.size(); i++)
        sum += datas.at(i);
    double mean =  sum / datas.size(); //均值

    double low_intensity = -1;
    double high_intensity = -1;

    bool low_found = false;
    bool high_found = false;

    std::cout << "min " << min << std::endl;
    std::cout << "max " << max << std::endl;
    std::cout << "mean " << mean << std::endl;

    double bin_width = (max-min)/HISTO_LENGTH;
//    std::cout << "bin_width: " << bin_width << std::endl;

    std::map<double,int> hist_map;
    for(unsigned int i = 0; i < dataHist.size(); i++)
        hist_map.insert(std::make_pair(dataHist.at(i), i));
    std::map<double,int>::reverse_iterator iter;
    iter = hist_map.rbegin();
    while(!low_found || !high_found)
    {
        int index = iter->second;
        double bin_edge = bin_width*double(index)+min;

        if (bin_edge > mean && !high_found)
        {
            high_found = true;
            high_intensity = bin_edge;
        }
        if (bin_edge < mean && !low_found)
        {
            low_found = true;
            low_intensity = bin_edge;
        }
        iter++;
    }


    std::cout << low_intensity << " " <<  high_intensity << std::endl;

    /// 画出直方图统计图
#if 0
    cv::Mat image2 = cv::Mat::zeros(400,400, CV_8UC3);
    for(int i=0;i<HISTO_LENGTH;i++){

        double height=dataHist[i];//计算高度
        //画出对应的高度图
        cv::rectangle(image2,cv::Point(i*2,400), cv::Point((i+1)*2 - 1, 400-height), CV_RGB(255,255,255));
    }
    cv::imshow("hist",image2);
    cv::waitKey(0);
#endif

    return Eigen::Vector2d(low_intensity, high_intensity);
}



