#pragma once
#include "Eigen/Core"


class BaseBoid
{
private:
	double r = 1.0; //color; red
	double g = 1.0; //color; green
	double b = 1.0; //color; blue
public:
	int id = -1; //id
	double x; //_x-position
	double y; //y-position
	double z; //z-position
	double angleY; //radian angle: 0 vector is (0, 1)
	double angleZ; //radian angle: 0 vector is (0, 1)
	Eigen::Vector3d vctr; //
	double speed; // speed
	int grid_y = -1; //grid address y
	int grid_x = -1; //grid address x
	int grid_z = -1; //grid address z
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * \brief 
	 * \param x 
	 * \param y 
	 * \param angleY 
	 * \param angleZ 
	 * \param speed 
	 * \param id 
	 */
	BaseBoid(double x = 0.0, double y = 0.0, double angleY = 0.0, double angleZ = 0.0, double speed = 0.0, int id = -1);

	BaseBoid(double _x, double _y, double _z, double _angleY, double _angleZ, double _speed, int _id);
	/**
	* \brief
	* \param red
	* \param green
	* \param blue
	*/
	void setColor(double red, double green, double blue);

	void drawBaseBoid();

	void updatePosition();

	/**
	* \brief
	* \param x
	* \param y
	* \param viewAngle
	* \return
	*/
	bool isVisible(double x, double y, double z, double viewAngle);
};
