#include "Vector.h"
#include <cmath>

Vector::Vector()
{
	X = 0;
	Y = 0;
}

Vector::Vector(double x, double y)
{
	this->X = x;
	this->Y = y;
}

double Vector::Length2()
{
	return X * X + Y * Y;
}

double Vector::Length()
{
	return sqrt(Length2());
}

Vector& Vector::Mult(double k)
{
	X *= k;
	Y *= k;
	return *this;
}

Vector & Vector::Normalize()
{
	double length = Length();
	X /= length;
	Y /= length;
	return *this;
}

Vector Vector::operator+(const Vector & v)
{
	return Vector(X + v.X, Y + v.Y);
}

double Vector::operator*(const Vector & v)
{
	return X * v.X + Y * v.Y;
}
