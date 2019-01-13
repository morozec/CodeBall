#include "BallEntity.h"

BallEntity::BallEntity(const model::Ball & ball)
{
	Radius = ball.radius;
	Position = Vector3D(ball.x, ball.y, ball.z);
	Velocity = Vector3D(ball.velocity_x, ball.velocity_y, ball.velocity_z);
	IsCollided = false;
	IsArenaCollided = false;
}

BallEntity::BallEntity(const BallEntity & ballEntity)
{
	Radius = ballEntity.Radius;
	Position = Vector3D(ballEntity.Position);
	Velocity = Vector3D(ballEntity.Velocity);
	IsCollided = ballEntity.IsCollided;
	IsArenaCollided = ballEntity.IsArenaCollided;
}

double BallEntity::GetMass()
{
	return Constants::Rules.BALL_MASS;
}

double BallEntity::GetRadiusChangeSpeed() const
{
	return 0.0;
}

void BallEntity::SetRadiusChangeSpeed(double value)
{
	throw "NOT SUPPORTED";
}

double BallEntity::GetArenaE()
{
	return Constants::Rules.BALL_ARENA_E;
}
