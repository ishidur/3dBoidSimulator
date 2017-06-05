#include "stdafx.h"
#include "BaseBoid.h"
#include <GL/glut.h>
#include <corecrt_math_defines.h>
#include "Direction.h"
#include "parameters.h" //import common parameters
#include <iostream>

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

void renderBoid()
{
	//	glRotated(90.0, 0.0, 1.0, 0.0);
	//	glutSolidCone(0.4 * BOID_SIZE * sqrt(3.0) / 2.0, BOID_SIZE, 10, 10);
	glutSolidTeapot(BOID_SIZE);
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

void BaseBoid::drawBaseBoid()
{
	GLfloat color[] = {GLfloat(r),GLfloat(g),GLfloat(b),1.0};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	glColor3d(r, g, b);
	glPushMatrix();
	glTranslated(x, y, z);
	glRotated(radianToDegree(angleY), 0.0, -1.0, 0.0);
	glRotated(radianToDegree(angleZ), 0.0, 0.0, 1.0);
	renderBoid();
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

double inner(Eigen::Vector3d a, Eigen::Vector3d b)
{
	return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

//currently not working
bool BaseBoid::isVisible(double _x, double _y, double _z, double _viewAngle)
{
	double dx = _x - x;
	double dy = _y - y;
	double dz = _z - z;
	//	Direction bDirection = Direction(dx, dy, dz);
	Eigen::Vector3d dist = Eigen::Vector3d(dx, dy, dz);
	double innertial = inner(vctr.normalized(), dist.normalized());
	double angle = acos(innertial);
	if (id == 0)
	{
		std::cout << "innertial: " << innertial << std::endl;
		std::cout << "angle: " << angle << std::endl;
	}
	if (angle > _viewAngle)
	{
		return false;
	}
	return true;
}
