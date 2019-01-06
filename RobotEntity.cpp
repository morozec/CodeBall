#include "RobotEntity.h"
#include "Constants.h"

RobotEntity::RobotEntity(const model::Robot & robot)
{
	Position = Vector3D(robot.x, robot.y, robot.z);
	Velocity = Vector3D(robot.velocity_x, robot.velocity_y, robot.velocity_z);
	Radius = robot.radius;
	Touch = robot.touch;
	if (Touch)
		TouchNormal = Vector3D(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z);
	Nitro = robot.nitro_amount;
}

RobotEntity::RobotEntity(const RobotEntity & robotEntity)
	: Entity(robotEntity)
{
	Position = Vector3D(robotEntity.Position);
	Velocity = Vector3D(robotEntity.Velocity);
	Radius = robotEntity.Radius;
	_radiusChangeSpeed =robotEntity.GetRadiusChangeSpeed();
	Touch = robotEntity.Touch;
	TouchNormal = robotEntity.TouchNormal;
	Nitro = robotEntity.Nitro;
	Action = robotEntity.Action;
}

RobotEntity::RobotEntity(const Vector3D & position, const Vector3D & velocity, double radius, bool touch, const Vector3D touchNormal, double nitro)
{
	Position = position;
	Velocity = velocity;
	Radius = radius;
	Touch = touch;
	TouchNormal = touchNormal;
	Nitro = nitro;
}

double RobotEntity::GetMass()
{
	return Constants::Rules.ROBOT_MASS;
}

double RobotEntity::GetRadiusChangeSpeed() const
{
	return _radiusChangeSpeed;
}

void RobotEntity::SetRadiusChangeSpeed(double value)
{
	_radiusChangeSpeed = value;
}

double RobotEntity::GetArenaE()
{
	return Constants::Rules.ROBOT_ARENA_E;
}
