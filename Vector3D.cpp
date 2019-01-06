#include "Vector3D.h"
#include <cmath>

Vector3D::Vector3D()
{
	X = 0;
	Y = 0;
	Z = 0;
}

Vector3D::Vector3D(double x, double y, double z)
{
	this->X = x;
	this->Y = y;
	this->Z = z;
}

Vector3D::Vector3D(const Vector3D & v)
{
	this->X = v.X;
	this->Y = v.Y;
	this->Z = v.Z;
}

double Vector3D::Length() const
{
	return sqrt(Length2());
}

double Vector3D::Length2() const
{
	return X*X+Y*Y+Z*Z;
}

void Vector3D::Normalize()
{
	double vLength = Length();
	X /= vLength;
	Y /= vLength;
	Z /= vLength;
}

void Vector3D::Sub(const Vector3D& v)
{
	X -= v.X;
	Y -= v.Y;
	Z -= v.Z;
}

void Vector3D::Add(const Vector3D& v)
{
	X += v.X;
	Y += v.Y;
	Z += v.Z;
}

void Vector3D::mult(double k)
{
	X *= k;
	Y *= k;
	Z *= k;
}

Vector3D Vector3D::operator+(const Vector3D & v)
{
	double newX = this->X + v.X;
	double newY = this->Y + v.Y;
	double newZ = this->Z + v.Z;
	return Vector3D(newX, newY, newZ);
}

Vector3D Vector3D::operator-(const Vector3D & v)
{
	double newX = this->X - v.X;
	double newY = this->Y - v.Y;
	double newZ = this->Z - v.Z;
	return Vector3D(newX, newY, newZ);
}

Vector3D Vector3D::operator*(double k)
{	
	return Vector3D(X*k, Y*k, Z*k);
}

double Vector3D::operator*(const Vector3D & v)
{
	return X * v.X + Y * v.Y + Z * v.Z;
}

double Vector3D::angleTo(const Vector3D& v)
{
	return acos(*this * v / Length() / v.Length());
}

