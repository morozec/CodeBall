#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _DANCALCULATOR_H_
#define _DANCALCULATOR_H_

#include "Vector.h"
#include "Vector3D.h"
#include "model/Arena.h"
#include "Dan.h"

class DanCalculator {
private:
	static const double Eps;
	static Vector3D _vector000;
	static Vector3D _vector010;
	static Vector3D _vector0M10;
	static Vector3D _vector0Ah0;
	static Vector3D _vectorAw200;
	static Vector3D _vector00Ad2Plus;
	static Vector3D _vector00M1;
	static Vector3D _vector00Ad2;
	static Vector3D _vectorAgw200;
	static Vector3D _vectorM100;
	static Vector3D _vector0Agh0;

	static Vector _topCornerO;
	static Vector _goalOuterCorner;
	static Vector _cornerO;

public:
	static void Init(const model::Arena& arena);
	static Dan Min(Dan& dan1, Dan dan2);
	static Dan GetDanToPlane(const Vector3D& point, const Vector3D& pointOnPlane, const Vector3D& planeNormal);
	static Dan GetDanToSphereInner(const Vector3D& point, const Vector3D& sphereCenter, double sphereRadius);
	static Dan GetDanToSphereOuter(const Vector3D& point, const Vector3D& sphereCenter, double sphereRadius);
	static Dan GetDanToArenaQuarter(const Vector3D& point, const model::Arena& arena);
	static Dan GetDanToArena(Vector3D point, const model::Arena& arena);
};

#endif