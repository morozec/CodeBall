#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _VECTOR3D_H_
#define _VECTOR3D_H_

struct Vector3D {
	double X;
	double Y;
	double Z;

	Vector3D();
	Vector3D(double x, double y, double z);
	Vector3D(const Vector3D& v);
	

	double Length() const;
	double Length2() const;
	void Normalize();
	void Sub(const Vector3D& v);
	void Add(const Vector3D& v);
	void mult(double k);
	
	Vector3D operator+(const Vector3D& v) const;
	Vector3D operator-(const Vector3D& v) const;
	Vector3D operator*(double k) const;
	double operator*(const Vector3D& v) const;

	double angleTo(const Vector3D& v) const;

};

#endif