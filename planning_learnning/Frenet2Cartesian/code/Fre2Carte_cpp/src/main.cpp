/*
 * @Date: 2021-03-14 06:29:39
 * @Author: Zhiqi Feng
 * @LastEditors: feng 
 * @LastEditTime: 2021-04-29 18:09:09
 * @FilePath: /Fre2Carte_cpp/src/main.cpp
 */

#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "eigen3/Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <ctime>
#include "spline.h"
#include "Waypoints.h"
#include "TrajLog.h"
#include "fstream"
#include "string"
#include "sstream"

using namespace std;
using namespace Eigen;
using namespace cv;
#define mapSize 800

void drawFigure(cv::Mat &figure, Pointxy ptr, Pointxy startPtr, CvScalar color) {
    Pointxy centerPtr(0, mapSize*0.5);
    Pointxy ptrDraw = (ptr - startPtr) * 3 + centerPtr;
    cv::circle(figure, cv::Point(ptr.x, mapSize - ptr.y), 1, color, -1);
}

Eigen::Vector4d cubicPolyCurve1d(double x0, double dx0, double x1, double dx1, double T) {
    Eigen::Vector4d coeffs;
    coeffs[0] = x0;
    coeffs[1] = dx0;
    double T2 = T*T;
    
    coeffs[2] = (3*x1 - T*dx1 - 3*coeffs[0] - 2*coeffs[1]*T)/T2;
    coeffs[3] = (dx1 - coeffs[1] - 2*coeffs[2]*T)/(3.0*T2);
    return coeffs;
}

double polyfit(Eigen::Vector4d coeffs, double t, int order) {
    if(order == 0)
        return coeffs[0] + coeffs[1]*t + coeffs[2]*t*t + coeffs[3]*t*t*t;
    if(order == 1)
        return coeffs[1] + 2*coeffs[2]*t+3*coeffs[3]*t*t;
    if(order == 2)
        return 2*coeffs[2]+6*coeffs[3]*t;
    if(order == 3)
        return 6*coeffs[3];
    else
        return 0.0;
}


int main() {
    Waypoints map;
    TrajLog trajLog = TrajLog("ego_sdxy", "baseline");
    trajLog.readCSVEgoSD();

    return 0;
}


// int main() {
//     Waypoints map;

//     cv::Mat figureCartesian = Mat::zeros(Size(mapSize, mapSize), CV_8UC3);
//     cv::Mat figureFrenet = Mat::zeros(Size(mapSize, mapSize), CV_8UC3);
//     figureCartesian.setTo(Scalar(255, 255, 255));
//     figureFrenet.setTo(Scalar(255, 255, 255));
    
    
//     double sLimit = 400;
//     double tLimit = 10.0;
//     Pointxy startPtrCartesian(map.GetXYInterpolated(400, 0));
//     Pointxy startPtrFrenet(400, 0);
//     Pointxy ptr;
//     // 画出原始轨迹图， s = 0->3000
//     for(double i = 400; i < 400+sLimit; i = i + 0.1) {
//         Pointxy ptrFrenet(i, 0);
//         ptr = map.GetXYInterpolated(ptrFrenet.x, ptrFrenet.y);
//         drawFigure(figureCartesian, ptr, startPtrCartesian, Scalar(0,0,255));
//         drawFigure(figureFrenet, ptrFrenet, startPtrFrenet, Scalar(0,0,255));


//         // 三个车道
//         ptrFrenet.y += 4;
//         ptr = map.GetXYInterpolated(ptrFrenet.x, ptrFrenet.y);
//         drawFigure(figureCartesian, ptr, startPtrCartesian, Scalar(0,0,0));
//         drawFigure(figureFrenet, ptrFrenet, startPtrFrenet, Scalar(0,0,0));
        
//         ptrFrenet.y += 4;
//         ptr = map.GetXYInterpolated(ptrFrenet.x, ptrFrenet.y);
//         drawFigure(figureCartesian, ptr, startPtrCartesian, Scalar(0,255,0));
//         drawFigure(figureFrenet, ptrFrenet, startPtrFrenet, Scalar(0,255,0));

//         ptrFrenet.y += 4;
//         ptr = map.GetXYInterpolated(ptrFrenet.x, ptrFrenet.y);
//         drawFigure(figureCartesian, ptr, startPtrCartesian, Scalar(255,0,0));
//         drawFigure(figureFrenet, ptrFrenet, startPtrFrenet, Scalar(255,0,0));
//     }
    
//     // 多项式轨迹
//     std::vector<Eigen::Vector4d> dCoeffs;
//     std::vector<Eigen::Vector4d> sCoeffs;
//     for(int i = 0; i < 10; ++i) {
//         dCoeffs.push_back(cubicPolyCurve1d(0, 0, i*4, 0, tLimit));
//         sCoeffs.push_back(cubicPolyCurve1d(startPtrFrenet.x, 20.0, startPtrFrenet.x+sLimit, 20.0, tLimit));
        
//         for(double t = 0; t <= tLimit; t += 0.01) {
//             double s = polyfit(sCoeffs.back(), t, 0);
//             double d = polyfit(dCoeffs.back(), t, 0);
//             Pointxy xyPtr = map.GetXYInterpolated(s, d);
//             drawFigure(figureFrenet, Pointxy(s, d), startPtrFrenet, Scalar(0,255,0));
//             drawFigure(figureCartesian, xyPtr, startPtrCartesian, Scalar(0,255,0));

//             if(i == 0) {
//                 std::array<double, 3> s_condition, d_condition;
//                 s_condition[0] = s;
//                 s_condition[1] = polyfit(sCoeffs.back(), t, 1);
//                 s_condition[2] = polyfit(sCoeffs.back(), t, 2);

//                 double dd = polyfit(dCoeffs.back(), t, 1);
//                 double ddd = polyfit(dCoeffs.back(), t, 2);
//                 d_condition[0] = d;
//                 d_condition[1] = dd/s_condition[1];
//                 d_condition[2] = (ddd*s_condition[1] - dd*s_condition[2])/(pow(s_condition[1], 3));

//                 if(std::isnan(d_condition[1])) {
//                     d_condition[1] = 0.0;
//                 }
//                 else if(std::isinf(d_condition[1])) {
//                     if(d_condition[1] < 0) {
//                         d_condition[1] = -1e6;
//                     }
//                     else {
//                         d_condition[1] = 1e6;
//                     }
//                 }

//                 if(std::isnan(d_condition[2])) {
//                     d_condition[2] = 0.0;
//                 }
//                 else if(std::isinf(d_condition[2])) {
//                     if(d_condition[2] < 0) {
//                         d_condition[2] = -1e6;
//                     }
//                     else {
//                         d_condition[2] = 1e6;
//                     }
//                 }

//                 double ptr_x;
//                 double ptr_y; 
//                 double ptr_theta; 
//                 double ptr_kappa;
//                 double ptr_v;
//                 double ptr_a;

//                 map.transFrenet2Cartesian(s, d, s_condition, d_condition, &ptr_x,
//                             &ptr_y, &ptr_theta, &ptr_kappa,
//                             &ptr_v, &ptr_a);
                
//                 drawFigure(figureCartesian, Pointxy(ptr_x, ptr_y), startPtrCartesian, Scalar(255,255,0));

//                 std::cout << ptr_a << "  " << ptr_v << "  " << ptr_kappa << std::endl;
//             }
//         }
//     }

//     cv::imshow("figureCartesian", figureCartesian);
//     cv::imshow("figureFrenet", figureFrenet);
//     cv::waitKey(0);


    
//     return 0;
// }

