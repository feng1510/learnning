#include "Waypoints.h"
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

using namespace std;

using Eigen::Vector2d;

Waypoints::Waypoints()
{
	// Waypoint map to read from
	const string map_file = "../data/highway_map.csv";

	ifstream in_map(map_file.c_str(), ifstream::in);

	Waypoint wp;
	string line;
	while (getline(in_map, line)) {
		istringstream iss(line);
		iss >> wp.x;
		iss >> wp.y;
		iss >> wp.s;
		iss >> wp.d_x;
		iss >> wp.d_y;
		m_waypoints.push_back(wp);
	}

	FitSpline();
}


Pointxy Waypoints::GetXYInterpolated(double s, double d) const
{
	s = NormalizeS(s);

	Pointxy pt(m_x_spline(s), m_y_spline(s));
	// normal vector
	Pointxy nv(-m_y_spline.deriv(1,s), m_x_spline.deriv(1,s));
	return pt + nv * d;
}


Vector2d Waypoints::CalcFrenet(const Pointxy& ptXY, double s_start) const
{
	// Gradient descent is used to find the pointxy (s,d) on the spline, which is closest to pointxy ptXY.
	// 运用梯度下降法寻找最接近ptXY的（s,d）
	const double eps = 1.0e-6;
	const double gamma = 0.001;
	const double precision = 1e-12;
	double s = s_start;
	double prev_step_size = s;

	while (prev_step_size > precision)
	{
		const auto prev_s = s;
		s -= gamma * ErrorDeriv(ptXY, prev_s);
		prev_step_size = std::abs(s - prev_s);
	}

	Vector2d p(2);
	p << ptXY.x, ptXY.y;

	const Vector2d p_spline(m_x_spline(s), m_y_spline(s));
	const Vector2d p_delta = (p - p_spline).array() / GetNormalAt(s).array();
	const double d = 0.5 * (p_delta(0) + p_delta(1));

	return Vector2d(s, d);
}

double Waypoints::ErrorDeriv(const Pointxy& pt, double s) const
{
	return -2. * (pt.x - m_x_spline(s)) * m_x_spline.deriv(1, s)
		- 2. * (pt.y - m_y_spline(s)) * m_y_spline.deriv(1, s);
}

Vector2d Waypoints::GetNormalAt(double s) const
{
	return Vector2d(-m_y_spline.deriv(1, s), m_x_spline.deriv(1, s));
}

void Waypoints::FitSpline()
{
	// Spline fitting
//	sort(m_waypoints.begin(), m_waypoints.end(), [](const Waypoint& l, const Waypoint& r) {return l.s < r.s; });

#if 1
	//------------------------------------
	// Fix of kink at the end of the track!
	Waypoint wpF = m_waypoints.front();
	Waypoint wpL = m_waypoints.back();

	Waypoint wp1 = wpL;
	wp1.s -= m_max_S;

	Waypoint wp2 = wpF;
	wp2.s += m_max_S;

	m_waypoints.insert(m_waypoints.begin(), wp1);
	m_waypoints.push_back(wp2);
	//------------------------------------
#endif


	vector<double> x, y, s;
	x.resize(m_waypoints.size());
	y.resize(m_waypoints.size());
	s.resize(m_waypoints.size());
	for (size_t i = 0; i < m_waypoints.size(); i++) {
		const auto& w = m_waypoints[i];
		x[i] = w.x;
		y[i] = w.y;
		s[i] = w.s;
	}

	m_x_spline.set_points(s, x);
	m_y_spline.set_points(s, y);
}

/**
  * @brief: 计算参参考轨迹的曲率
  * @description: 
  * @param {double} dx
  * @param {double} ddx
  * @param {double} dy
  * @param {double} ddy
  * @return {*}
  */
double Waypoints::ComputeCurvature(double dx, double ddx, double dy, double ddy) {
	double a = dx*ddy - dy*ddx;
	double norm_square = dx*dx+dy*dy;
	double norm = sqrt(norm_square);
	double b = norm*norm_square;
	if(b < 1e-6 && b > -1e-6) {
		return 1e6;
	}
	return a/b;
}
double Waypoints::ComputeCurvatureDerivative(double dx, double ddx, double dddx, double dy, double ddy, double dddy) {
	double a = dx*ddy-dy*ddx;
	double b = dx*dddy-dy*dddx;
	double c = dx*ddx+dy*ddy;
	double d = dx*dx+dy*dy;
	if(d < 1e-6 && d > -1e-6) {
		return 1e6;
	}
	return (b*d-3.0*a*c)/(d*d*d);
}

/**
 * @brief: 获取参考线上ptr点的曲率和曲率微分
 * @description: 
 * @param {Pointxy} ptr
 * @return {*}
 */
Eigen::Vector2d Waypoints::getCurvature(double s) {
	double dx, ddx, dddx, dy, ddy, dddy;
	dx 		= m_x_spline.deriv(1, s);
	ddx 	= m_x_spline.deriv(2, s);
	dddx 	= m_x_spline.deriv(3, s);
	
	dy 		= m_y_spline.deriv(1, s);
	ddy 	= m_y_spline.deriv(2, s);
	dddy 	= m_y_spline.deriv(3, s);

	Eigen::Vector2d kappa_r_dr(0.0, 0.0);
	kappa_r_dr(0) = ComputeCurvature(dx,ddx,dy,ddy);
	kappa_r_dr(1) = ComputeCurvatureDerivative(dx,ddx, ddx,dy,ddy,dddy);
	return kappa_r_dr;
}

/**
* @brief: 计算Frenet到笛卡尔坐标系的转换
* @description: 
* @param {*}
* @return {*}
*/
void Waypoints::transFrenet2Cartesian(double s, double d, 
						const std::array<double, 3>& s_condition,
						const std::array<double, 3>& d_condition, double* const ptr_x,
						double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
						double* const ptr_v, double* const ptr_a) {

	Eigen::Vector2d kappa = getCurvature(s);
	double rx = m_x_spline(s);
	double ry = m_y_spline(s);
	double rtheta = std::atan2(m_y_spline.deriv(1, s), m_x_spline.deriv(1, s));
	frenet_to_cartesian(s, rx, ry, rtheta, kappa(0), kappa(1), s_condition, d_condition, ptr_x, ptr_y, ptr_theta, ptr_kappa, ptr_v, ptr_a);

}
void Waypoints::frenet_to_cartesian(const double rs, const double rx, const double ry, const double rtheta,
						const double rkappa, const double rdkappa,
						const std::array<double, 3>& s_condition,
						const std::array<double, 3>& d_condition, double* const ptr_x,
						double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
						double* const ptr_v, double* const ptr_a) {
							
	// const double cos_theta_r = std::cos(rtheta);
	// const double sin_theta_r = std::sin(rtheta);

	const double cos_theta_r = m_x_spline.deriv(1, rs);
	const double sin_theta_r = m_y_spline.deriv(1, rs);

	*ptr_x = rx - sin_theta_r * d_condition[0];
	*ptr_y = ry + cos_theta_r * d_condition[0];

	const double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

	const double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
	const double delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
	const double cos_delta_theta = std::cos(delta_theta);

	*ptr_theta = NormalizeAngle(delta_theta + rtheta);

	const double kappa_r_d_prime =
		rdkappa * d_condition[0] + rkappa * d_condition[1];
	*ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
					cos_delta_theta * cos_delta_theta) /
						(one_minus_kappa_r_d) +
					rkappa) *
				cos_delta_theta / (one_minus_kappa_r_d);
	
	std::cout << d_condition[2] << " " << kappa_r_d_prime << "  " << tan_delta_theta << "  " << cos_delta_theta << "  " << cos_delta_theta << "  "  << one_minus_kappa_r_d << "  "  << rkappa << "  " << cos_delta_theta << " " << one_minus_kappa_r_d << "  ";

	const double d_dot = d_condition[1] * s_condition[1];
	*ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
							s_condition[1] * s_condition[1] +
						d_dot * d_dot);

	const double delta_theta_prime =
		one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;

	*ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
			s_condition[1] * s_condition[1] / cos_delta_theta *
				(d_condition[1] * delta_theta_prime - kappa_r_d_prime);
}

double Waypoints::NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}