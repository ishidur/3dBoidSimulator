#pragma once
#include "Eigen/Core"

class Direction
{
public:
	double angle_y;
	double angle_z;
	double x;
	double y;
	double z;
	Eigen::Vector3d vector;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * \brief 
	 * \param angle_y 
	 * \param angle_z 
	 */
	Direction(double angle_y, double angle_z);
	/**
	 * \brief 
	 * \param x 
	 * \param y 
	 * \param z 
	 */
	Direction(double x, double y, double z);
	/**
	* \brief
	* \param vector
	*/
	Direction(Eigen::Vector3d& vector);
};
