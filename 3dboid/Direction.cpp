#include "stdafx.h"
#include "Direction.h"
#include <iostream>

Direction::Direction(double _angleY, double _angleZ)
{
	angleY = _angleY;
	angleZ = _angleZ;
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
	angleZ = asin(_y / dist);
	double distXZ = sqrt(_x * _x + _z * _z);
	if (_z >= 0.0)
	{
		angleY = acos(_y / distXZ);
	}
	else
	{
		angleY = -acos(_y / distXZ);
	}
}

Direction::Direction(Eigen::Vector3d& v)
{
	double dist = v.norm();
	x = v.x() / dist;
	y = v.y() / dist;
	z = v.z() / dist;
	vector = v.normalized();
	angleZ = asin(v.y() / dist);
	double distXZ = sqrt(x * x + z * z);

	if (z >= 0.0)
	{
		angleY = acos(x / distXZ);
	}
	else
	{
		angleY = -acos(x / distXZ);
	}
}
