#pragma once

#ifndef TYPES_H
#define TYPES_H

#include "eigen3/Eigen/Dense"
#include <limits>
#include <opencv2/opencv.hpp>
//#include "Decision.h"

//#define VERBOSE_NEXT_XY
//#define VERBOSE_BEST_TRAJECTORY
//#define VERBOSE_TRAJECTORIES
//#define VERBOSE_OTHER_CARS_CHECK_TRANSFORM
#define VERBOSE_OTHER_LEADING_CARS
//#define VERBOSE_OTHER_IGNORED_CARS

//#define VERBOSE_STATE

using namespace Eigen;
using namespace cv;
using namespace std;

struct Pointxy
{
	double x;
	double y;

	Pointxy(double x = 0, double y = 0) : x(x), y(y) {}

	//自动操作符
	Pointxy& operator*(double n) { x *= n; y *= n; return *this; }
	Pointxy& operator+(const Pointxy& r) {
		if (this != &r) {
			x += r.x;
			y += r.y;
		}
		return *this;
	}

	Pointxy& operator-(const Pointxy& r) {
		if (this != &r) {
			x -= r.x;
			y -= r.y;
		}
		return *this;
	}
};


struct Waypoint
{
	double x;
	double y;
	double s;
	double d_x;
	double d_y;

	Waypoint(double x = 0, double y = 0, double s = 0, double d_x = 0, double d_y = 0)
		: x(x), y(y), s(s), d_x(d_x), d_y(d_y) {}
};


struct CarLocalizationData
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;

	CarLocalizationData(double x = 0, double y = 0, double s = 0, double d = 0, double yaw = 0, double speed = 0)
		: x(x), y(y), s(s), d(d), yaw(yaw), speed(speed) {}
};


struct SensorFusionData
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
};

//feng

struct dsv
{
	double d;
	double s;
	double v;
	dsv(double d = 0, double s = 0, double v = 0):d(d), s(s), v(v){}
};

//定义采样点的数据结构
struct Sample_point
{
	int id;
	int lane_number;
	double x;
	double y;
	double d;       //这里的d为正数
	double s;
	double vx;
	double vy;
	int block_id;
	double cost_dis;
	double cost_pre[3];
	bool best;
	Eigen::VectorXd change_cost {Eigen::VectorXd::Zero(4)};  //换道目标点的代价
	Sample_point(int id = 0, int lane_number = 0, double x = 0, double y = 0, double d = 0, double s = 0, double vx = 0,
				 double vy = 0, int block_id = 0, double cost_dis = 0, bool best = false)
				 : id(id), lane_number(lane_number), x(x), y(y), d(d), s(s), vx(vx), vy(vy), block_id(block_id), cost_dis(cost_dis), best(best){}
};


struct Search_sequ
{
    std::vector<Point> search_list;
    std::vector<vector<Point> > points_list;
};

struct Blank_block
{
    int id;             //一条车道上的序列，由主车向车前方排序 从0开始0 1 2 。。。
    int d_id;           //block所在的车道序列， 分别为 3,2,1
    double d;
    double s;
    double length;
	bool egoFlag;
    Eigen::VectorXd change_cost {Eigen::VectorXd::Zero(4)};  //换道目标点的代价
    Eigen::VectorXd change_score {Eigen::VectorXd::Zero(4)};
    double cost_sum;
    double score_sum;
    Blank_block(int id = -1, int d_id = 0, double d = 0, double s = 0, double length = 0, bool egoFlag = false, double cost_sum = 0, double score_sum = 0) : id(id), d_id(d_id), d(d), s(s), length(length), egoFlag(egoFlag), cost_sum(cost_sum), score_sum(score_sum) {}
};



#endif // TYPES_H
