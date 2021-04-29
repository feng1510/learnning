/*
 * @Date: 2021-03-24 00:43:39
 * @Author: Zhiqi Feng
 * @LastEditors: feng 
 * @LastEditTime: 2021-03-26 22:27:46
 * @FilePath: /Fre2Carte_cpp/include/TrajLog.h
 */
#pragma once

#include "Types.h"
#include "Waypoints.h"
#include "spline.h"
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <vector>
#include "eigen3/Eigen/Dense"

using namespace std;


class TrajLog {
    public:
        TrajLog(std::string logTime_, std::string logType);
        ~TrajLog();
    public:
        double polyfit(Eigen::VectorXd coeffs, double T, int order);
        bool readCSV();
        bool readCSVEgoSD();
        void calculateAccCurvature(double duration, int index);
        std::string getTime();

    public:
        Waypoints *map;
        std::string logTime;
        std::string logPath;
        TrajLogData sdData;
        ofstream *minMaxAccCurvature;
        ofstream *egoAccCurvature;
        ofstream *egoSD;
        
};
