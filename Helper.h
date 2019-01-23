#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _HELPER_H_
#define _HELPER_H_

#include "Vector3D.h"
#include "model/Action.h"
#include "model/Robot.h"
#include "model/Ball.h"

class Helper {
public:
	inline static double EPS = 1E-5;
	static double Clamp(double value, double minValue, double maxValue);
	static Vector3D Clamp(Vector3D v, double maxLength);
	static Vector3D Clamp2(Vector3D v, double maxLength2);
	static Vector3D Clamp(Vector3D& v, double maxLength, double maxLength2);
	static double GetLength2(const Vector3D& v1, const Vector3D& v2);
	static double GetLength(const Vector3D& v1, const Vector3D& v2);
	static Vector3D GetTargetVelocity(double x0, double y0, double z0,
		double x1, double y1, double z1, double velocity);
	static Vector3D GetTargetVelocity(const Vector3D& startPos, const Vector3D& targetPos, double velocity);
	static Vector3D GetAcionTargetVelocity(const model::Action& action);

	static Vector3D GetRobotPosition(const model::Robot robot);
	static Vector3D GetRobotVelocity(const model::Robot robot);

	static Vector3D GetBallPosition(const model::Ball ball);
	static Vector3D GetBallVelocity(const model::Ball ball);
};

#endif