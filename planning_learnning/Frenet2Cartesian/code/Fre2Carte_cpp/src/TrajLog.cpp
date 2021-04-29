/*
 * @Date: 2021-03-24 00:53:22
 * @Author: Zhiqi Feng
 * @LastEditors: feng 
 * @LastEditTime: 2021-03-26 23:09:34
 * @FilePath: /Fre2Carte_cpp/src/TrajLog.cpp
 */

#include "TrajLog.h"

using namespace std;

TrajLog::TrajLog(std::string logTime_, std::string logType) {
    logTime = logTime_;
    logPath = "../data/";
    logPath.append(logTime_);
    if(logType == "trajLog") {
        logPath.append("-trajLog.csv");
        
    }
    else if(logType == "baseline") {
        logPath.append(".csv");
    }
    sdData          = TrajLogData(0.0);
    map             = new Waypoints();
    minMaxAccCurvature  = new ofstream("../data/" + logTime_ + "_" + getTime() + "-minMaxAccCurvature.csv");
    egoAccCurvature     = new ofstream("../data/" + logTime_ + "_" + getTime() + "-egoAccCurvature.csv");
    egoSD               = new ofstream("../data/" + logTime_ + "_" + getTime() + "-egoSD.csv");
    *minMaxAccCurvature << "minAcc" << ", " << "maxAcc" << ", " << "minKappa" << ", " << "maxKappa" << std::endl;
    *egoAccCurvature << "Acc" << ", " << "Kappa" << std::endl;
    *egoSD << "s" << ", " << "d" << std::endl;
    std::cout << "Object TrajLog is being created" << std::endl;
}

TrajLog::~TrajLog() {
    minMaxAccCurvature->close();
    egoAccCurvature->close();
    egoSD->close();
    delete map;
    delete minMaxAccCurvature;
    delete egoAccCurvature;
    std::cout << "Object TrajLog is being deleted" << std::endl;
}

double TrajLog::polyfit(Eigen::VectorXd coeffs, double T, int order) {
    double T2 = T*T;
    double T3 = T2*T;
    double T4 = T2*T2;
    double T5 = T2*T3;
    if(order == 0) {
        return coeffs[0] + coeffs[1]*T + coeffs[2]*T2 + coeffs[3]*T3 + coeffs[4]*T4 + coeffs[5]*T5;
    }
    else if (order == 1) {
        return coeffs[1] + coeffs[2]*T*2 + coeffs[3]*T2*3 + coeffs[4]*T3*4 + coeffs[5]*T4*5;
    }
    else if (order == 2) {
        return coeffs[2]*2 + coeffs[3]*T*2*3 + coeffs[4]*T2*3*4 + coeffs[5]*T3*4*5;
    }
    else if(order == 3) {
        return coeffs[3]*2*3 + coeffs[4]*T*2*3*4 + coeffs[5]*T2*3*4*5;
    }
    else {
        std::cout << "**Illegal operation! Error with variable--order**" << std::endl;
        return 0;
    }
}

bool TrajLog::readCSV() {
    ifstream logData(logPath.c_str(), ifstream::in);
    if(!logData) {
        std::cout << "open logData fail" << std::endl;
        return false;
    }
    string line;
    int count = 2;
    getline(logData, line); // 先去掉第一行头部说明
	while (getline(logData, line)) {
		istringstream iss(line);
        string number;
        for (int i = 0; i < 6; i++) {
            getline(iss, number, ',');
            sdData.sData[i] = std::atof(number.c_str());
        }
        for (int i = 0; i < 6; i++) {
            getline(iss, number, ',');
            sdData.dData[i] = std::atof(number.c_str());
        }
        getline(iss, number, ',');
        sdData.duration = std::atof(number.c_str());
        calculateAccCurvature(sdData.duration, count);
        count += 1;
        // if(count > 100) {
        //     return true;
        // }
	}
}

bool TrajLog::readCSVEgoSD() {
    ifstream logData(logPath.c_str(), ifstream::in);
    if(!logData) {
        std::cout << "open logData fail" << std::endl;
        return false;
    }
    string line;
    int count = 2;
    Pointxy vlocalxy;
    double s;
    // getline(logData, line); // 先去掉第一行头部说明
	while (getline(logData, line)) {
		istringstream iss(line);
        string number;
        getline(iss, number, ',');
        getline(iss, number, ',');
        s = std::atof(number.c_str());
        getline(iss, number, ',');
        getline(iss, number, ',');
        vlocalxy.x = std::atof(number.c_str());
        getline(iss, number, ',');
        vlocalxy.y = std::atof(number.c_str());
        Eigen::Vector2d vLocalSD = map->CalcFrenet(vlocalxy, s);
        *egoSD << vLocalSD(0) << "," << vLocalSD(1) << std::endl;
	}
}

void TrajLog::calculateAccCurvature(double duration, int index) {
    double t = 0.0;
    std::array<double, 3> s_condition, d_condition;
    double s, d;
    double accMin = 1e6, accMax = -1e6, kappaMin = 1e6, kappaMax = -1e6;
    for(double t = 0; t < duration; t+=0.1) {
        s = polyfit(sdData.sData, t, 0);
        d = polyfit(sdData.dData, t, 0);
        s_condition[0]  = s;
        s_condition[1]  = polyfit(sdData.sData, t, 1);
        s_condition[2]  = polyfit(sdData.sData, t, 2);

        double dd       = polyfit(sdData.dData, t, 1);
        double ddd      = polyfit(sdData.dData, t, 2);
        d_condition[0]  = d;
        d_condition[1]  = dd/s_condition[1];
        d_condition[2]  = (ddd*s_condition[1] - dd*s_condition[2])/(std::pow(s_condition[1], 3));

        if(std::isnan(d_condition[1])) {
            d_condition[1] = 0.0;
        }
        else if(std::isinf(d_condition[1])) {
            if(d_condition[1] < 0) {
                d_condition[1] = -1e10;
            }
            else {
                d_condition[1] = 1e10;
            }
        }

        if(std::isnan(d_condition[2])) {
            d_condition[2] = 0.0;
        }
        else if(std::isinf(d_condition[2])) {
            if(d_condition[2] < 0) {
                d_condition[2] = -1e10;
            }
            else {
                d_condition[2] = 1e10;
            }
        }

        double ptr_x;
        double ptr_y; 
        double ptr_theta; 
        double ptr_kappa;
        double ptr_v;
        double ptr_a;
        map->transFrenet2Cartesian(s, d, s_condition, d_condition, &ptr_x,
                                &ptr_y, &ptr_theta, &ptr_kappa,
                                &ptr_v, &ptr_a);
        accMin = std::min(accMin, ptr_a);
        accMax = std::max(accMax, ptr_a);
        kappaMin = std::min(kappaMin, ptr_kappa);
        kappaMax = std::max(kappaMax, ptr_kappa);
        if(t == 0) {
            *egoAccCurvature << ptr_a << ", " << ptr_kappa << std::endl;
        }
        // std::cout << "Acc Curvature*****   " << ptr_a << " m/s^2    " << ptr_kappa << " 1/m" <<std::endl;
        // if(std::abs(ptr_a) > 5 || std::abs(ptr_kappa) > (0.2*0.2)) {
            // std::cout << "Acc Curvature ERROR*****   " << index << "--    " << ptr_a << " m/s^2    " << ptr_kappa << " 1/m" <<std::endl;
            // std::cout << sdData.sData << std::endl << sdData.dData << std::endl;
            // std::cout << s << " " << d << " " << ptr_x << " " << ptr_y <<std::endl;
            // std::cout << s_condition[0] << "  " << d_condition[0] << std::endl;
            // std::cout << s_condition[1] << "  " << d_condition[1] << std::endl;
            // std::cout << s_condition[2] << "  " << d_condition[2] << std::endl;
        // }
    }
    *minMaxAccCurvature << accMin << ", " << accMax << ", " << kappaMin << ", " << kappaMax << std::endl;
}

std::string TrajLog::getTime() {
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}