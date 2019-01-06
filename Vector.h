#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _VECTOR_H_
#define _VECTOR_H_

struct Vector
{
	double X;
	double Y;

	Vector();
	Vector(double x, double y);

	double Length2();
	double Length();
	Vector& Mult(double k);
	Vector& Normalize();

	Vector operator+(const Vector& v);
	double operator*(const Vector& v);
};

#endif