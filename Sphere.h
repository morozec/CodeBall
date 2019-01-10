#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _SPHERE_H_
#define _SPHERE_H_

struct Sphere
{
public:
	double x;
	double y;
	double z;
	double radius;
	double r;
	double g;
	double b;
	double a;

	Sphere(double x, double y, double z, double radius, double r, double g, double b, double a)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->radius = radius;
		this->r = r;
		this->g = g;
		this->b = b;
		this->a = a;
	}		
};

#endif
