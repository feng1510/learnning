/*
 * @Date: 2021-03-14 06:35:00
 * @Author: Zhiqi Feng
 * @LastEditors: feng 
 * @LastEditTime: 2021-03-15 05:40:43
 * @FilePath: /Fre2Carte_cpp/include/Waypoints.h
 */
#pragma once

#include "Types.h"
#include "spline.h"
#include <vector>
#include "eigen3/Eigen/Dense"


class Waypoints
{
public:
	Waypoints();

	// Transform from Frenet s,d coordinates to Cartesian x,y using bspline for interpolation.
	//B样条曲线插值
	Pointxy GetXYInterpolated(double s, double d) const;

	// Returns pointxy [s, d] on the spline (Highway-Map) which is the closest pointxy to the (x,y).
	// s_start defines the initial position on the spline.
	Eigen::Vector2d CalcFrenet(const Pointxy& ptXY, double s_start) const;

	double max_s() const { return m_max_S; }

private:
	void FitSpline();
	double NormalizeS(double s) const;
	// Calculates derivative d/ds of error function [(x - x_spline)^2 + (y - y_spline)^2]
	// 计算误差函数的导数
	double ErrorDeriv(const Pointxy& pt, double s) const;
	// Returns normal to the spline at pointxy s.
	Eigen::Vector2d GetNormalAt(double s) const;
	
	/**
	 * @brief: 计算参参考轨迹的曲率
	 * @description: 
	 * @param {double} dx
	 * @param {double} ddx
	 * @param {double} dy
	 * @param {double} ddy
	 * @return {*}
	 */
	double ComputeCurvature(double dx, double ddx, double dy, double ddy);
	double ComputeCurvatureDerivative(double dx, double ddx, double dddx, double dy, double ddy, double dddy);

public:
	/**
	 * @brief: 获取参考线上ptr点的曲率和曲率微分
	 * @description: 
	 * @param {Pointxy} ptr
	 * @return {*}
	 */
	Eigen::Vector2d getCurvature(double s);

	/**
	* @brief: 计算Frenet到笛卡尔坐标系的转换
	* @description: 
	* @param {*}
	* @return {*}
	*/
	// Notations:
	// s_condition = [s, s_dot, s_ddot]
	// s: longitudinal coordinate w.r.t reference line.
	// s_dot: ds / dt
	// s_ddot: d(s_dot) / dt
	// d_condition = [d, d_prime, d_pprime]
	// d: lateral coordinate w.r.t. reference line
	// d_prime: dd / ds
	// d_pprime: d(d_prime) / ds
	// l: the same as d.

	void transFrenet2Cartesian(double s, double d, 
						const std::array<double, 3>& s_condition,
						const std::array<double, 3>& d_condition,double* const ptr_x,
						double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
						double* const ptr_v, double* const ptr_a);
	void frenet_to_cartesian(	const double rs, const double rx, const double ry, const double rtheta,
								const double rkappa, const double rdkappa,
								const std::array<double, 3>& s_condition,
								const std::array<double, 3>& d_condition, double* const ptr_x,
								double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
								double* const ptr_v, double* const ptr_a);
	double NormalizeAngle(const double angle);

private:
	// The max s value before wrapping around the track back to 0  定义最大的s,回到圆环起点
	const double m_max_S = 6945.554;

	//定义一个vector存储waypoint
	std::vector<Waypoint> m_waypoints;
	tk::spline m_x_spline; // x(s)
	tk::spline m_y_spline; // y(s)
};

//normal一下s的值，使得s的值在正常范围内
inline double Waypoints::NormalizeS(double s) const
{
	return (s > m_max_S) ? (s -= m_max_S) : (s < -m_max_S ? (s += m_max_S) : s);
}
