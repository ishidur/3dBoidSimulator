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
	double angle_y; //radian angle: 0 vector is (0, 1)
	double angle_z; //radian angle: 0 vector is (0, 1)
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
	 * \param angle_y 
	 * \param angle_z 
	 * \param speed 
	 * \param id 
	 */
	BaseBoid(double x = 0.0, double y = 0.0, double _z = 0.0, double angle_y = 0.0, double angle_z = 0.0, double speed = 0.0,
	         int id = -1);

	/**
	* \brief
	* \param red
	* \param green
	* \param blue
	*/
	void set_color(double red, double green, double blue);

	void draw_base_boid();

	void update_position();

	/**
	* \brief
	* \param x
	* \param y
	* \param view_angle
	* \return
	*/
	bool is_visible(double x, double y, double z, double view_angle);
};
