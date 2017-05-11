#pragma once
#include "Eigen/Core"

class Direction
{
public:
	double angleY;
	double angleZ;
	double x;
	double y;
	double z;
	Eigen::Vector3d vector;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * \brief 
	 * \param angleY 
	 * \param angleZ 
	 */
	Direction(double angleY, double angleZ);
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
