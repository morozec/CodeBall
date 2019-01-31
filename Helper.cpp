#include "Helper.h"
#include <cmath>

double Helper::Clamp(double value, double minValue, double maxValue)
{
	if (value < minValue) return minValue;
	if (value > maxValue) return maxValue;
	return value;
}

Vector3D Helper::Clamp(Vector3D v, double maxLength)
{
	double vLength = v.Length();
	if (vLength > maxLength)
		v.mult(maxLength / vLength);
	return v;
}

Vector3D Helper::Clamp2(Vector3D v, double maxLength2)
{
	double vLength2 = v.Length2();
	if (vLength2 < maxLength2) return v;
	v.mult(sqrt(maxLength2 / vLength2));
	return v;
}

Vector3D Helper::Clamp(Vector3D & v, double maxLength, double maxLength2)
{
	if (v.Length2() > maxLength2)
		v.mult(maxLength / v.Length());
	return v;
}

double Helper::GetLength2(const Vector3D & v1, const Vector3D & v2)
{
	return (v1.X - v2.X)*(v1.X - v2.X) + (v1.Y - v2.Y)*(v1.Y - v2.Y) + (v1.Z - v2.Z)*(v1.Z - v2.Z);
}

double Helper::GetLength(const Vector3D & v1, const Vector3D & v2)
{
	return sqrt(GetLength2(v1, v2));
}

Vector3D Helper::GetTargetVelocity(double x0, double y0, double z0, double x1, double y1, double z1, double velocity)
{
	double dx = x1 - x0;
	double dy = y1 - y0;
	double dz = z1 - z0;

	double length = sqrt(dx * dx + dy * dy + dz * dz);
	if (abs(length) < EPS)
		return Vector3D(0, 0, 0);
	dx = dx / length * velocity;
	dy = dy / length * velocity;
	dz = dz / length * velocity;
	return Vector3D(dx, dy, dz);
}

Vector3D Helper::GetTargetVelocity(const Vector3D & startPos, const Vector3D & targetPos, double velocity)
{
	double dx = targetPos.X - startPos.X;
	double dy = targetPos.Y - startPos.Y;
	double dz = targetPos.Z - startPos.Z;

	double length = sqrt(dx * dx + dy * dy + dz * dz);
	if (abs(length) < EPS)
		return Vector3D(0, 0, 0);
	dx = dx / length * velocity;
	dy = dy / length * velocity;
	dz = dz / length * velocity;
	return Vector3D(dx, dy, dz);
}

Vector3D Helper::GetAcionTargetVelocity(const model::Action & action)
{
	return Vector3D(
		action.target_velocity_x,
		action.target_velocity_y,
		action.target_velocity_z);
}

Vector3D Helper::GetRobotPosition(const model::Robot robot)
{
	return Vector3D(robot.x, robot.y, robot.z);
}

Vector3D Helper::GetRobotVelocity(const model::Robot robot)
{
	return Vector3D(robot.velocity_x, robot.velocity_y, robot.velocity_z);
}

Vector3D Helper::GetBallPosition(const model::Ball ball)
{
	return Vector3D(ball.x, ball.y, ball.z);
}

Vector3D Helper::GetBallVelocity(const model::Ball ball)
{
	return Vector3D(ball.velocity_x, ball.velocity_y, ball.velocity_z);
}

double Helper::GetDistToLine(double x1, double z1, double x2, double z2, double x0, double z0)
{
	return abs((z2 - z1)*x0 - (x2 - x1)*z0 + x2 * z1 - z2 * x1) / sqrt((z2 - z1)*(z2 - z1) + (x2 - x1)*(x2 - x1));
}


