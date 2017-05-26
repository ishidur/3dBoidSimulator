#include "stdafx.h"
#include "BaseBoid.h"
#include <GL/glut.h>
#include <corecrt_math_defines.h>
#include "Direction.h"
#include "parameters.h" //import common parameters
#include <iostream>

//‹«ŠEðŒ: •Ç
double checkBoundary(double pos)
{
	if (pos > BOUNDARY - WALL_SIZE)
	{
		pos = BOUNDARY - WALL_SIZE;
	}
	else if (pos < -BOUNDARY + WALL_SIZE)
	{
		pos = -BOUNDARY + WALL_SIZE;
	}
	return pos;
}

double radianToDegree(double rad)
{
	return rad * 180.0 / M_PI;
}

BaseBoid::BaseBoid(double _x, double _y, double _z, double _angleY, double _angleZ, double _speed, int _id)
{
	id = _id;
	x = _x;
	y = _y;
	z = _z;
	angleY = _angleY;
	angleZ = _angleZ;
	speed = _speed;
	vctr = Eigen::Vector3d(cos(_angleY) * cos(_angleZ) * _speed, sin(_angleZ) * _speed, sin(_angleY) * cos(_angleZ) * _speed);
}

void BaseBoid::setColor(double red, double green, double blue)
{
	r = red;
	g = green;
	b = blue;
}

void BaseBoid::drawBaseBoid() //TODO:’¹‚ç‚µ‚­
{
	GLfloat color[] = {GLfloat(r),GLfloat(g),GLfloat(b),1.0};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	glColor3d(r, g, b);
	glPushMatrix();
	glTranslated(x, y, z);
	glRotated(radianToDegree(angleY), 0.0, -1.0, 0.0);
	glRotated(radianToDegree(angleZ), 0.0, 0.0, 1.0);
	glutWireCone(-0.4 * BOID_SIZE * sqrt(3.0) / 2.0, BOID_SIZE, 10, 10);
	glPopMatrix();
}

void BaseBoid::updatePosition()
{
	x += vctr.x() * FLAME_RATE / 1000.0;
	y += vctr.y() * FLAME_RATE / 1000.0;
	z += vctr.z() * FLAME_RATE / 1000.0;
	x = checkBoundary(x);
	y = checkBoundary(y);
	z = checkBoundary(z);
}

//currently not working
bool BaseBoid::isVisible(double _x, double _y, double _z, double _viewAngle)
{
	//	double dx = _x - x;
	//	double dy = _y - y;
	//	Direction bDirection = Direction(dx, dy);
	//	double maxAngle = angleY + _viewAngle;
	//	double minAngle = angleY - _viewAngle;
	//
	//	bool max = maxAngle > M_PI ? bDirection.angle > maxAngle - 2.0 * M_PI : bDirection.angle > maxAngle;
	//	bool min = minAngle < -M_PI ? bDirection.angle < minAngle + 2.0 * M_PI : bDirection.angle < minAngle;
	//	if (maxAngle <= M_PI && minAngle > -M_PI)
	//	{
	//		if (max || min)
	//		{
	//			return false;
	//		}
	//	}
	//	else
	//	{
	//		if (max && min)
	//		{
	//			return false;
	//		}
	//	}
	return true;
}
