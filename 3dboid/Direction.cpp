#include "stdafx.h"
#include "Direction.h"
#include <iostream>

Direction::Direction(double _angleY, double _angleZ)
{
	angle_y = _angleY;
	angle_z = _angleZ;
	x = cos(_angleY) * cos(_angleZ);
	y = sin(_angleZ);
	z = sin(_angleY) * cos(_angleZ);
	vector = Eigen::Vector3d(x, y, z);
}

Direction::Direction(double _x, double _y, double _z)
{
	double dist = sqrt(_x * _x + _y * _y + _z * _z);
	x = _x / dist;
	y = _y / dist;
	z = _z / dist;
	vector = Eigen::Vector3d(x, y, z);
	angle_z = asin(_y / dist);
	double distXZ = sqrt(_x * _x + _z * _z);
	if (_z >= 0.0) { angle_y = acos(_y / distXZ); }
	else { angle_y = -acos(_y / distXZ); }
}

Direction::Direction(Eigen::Vector3d& v)
{
	double dist = v.norm();
	x = v.x() / dist;
	y = v.y() / dist;
	z = v.z() / dist;
	vector = v.normalized();
	angle_z = asin(v.y() / dist);
	double distXZ = sqrt(x * x + z * z);

	if (z >= 0.0) { angle_y = acos(x / distXZ); }
	else { angle_y = -acos(x / distXZ); }
}
