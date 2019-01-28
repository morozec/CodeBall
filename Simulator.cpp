#include "Simulator.h"
#include "Helper.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <functional>
#include <iostream>
#include <optional>

const double Simulator::Eps = 1E-3;
const double Simulator::Eps2 = 1E-6;

double Simulator::GetRobotRadius(double jumpSpeed)
{
	model::Rules rules = Constants::Rules;
	return rules.ROBOT_MIN_RADIUS + (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * jumpSpeed /
		rules.ROBOT_MAX_JUMP_SPEED;
}

void Simulator::CollideEntities(Entity & a, Entity & b, double hitE)
{
	Vector3D deltaPosition = b.Position - a.Position;
	double distance = deltaPosition.Length();
	double penetration = a.Radius + b.Radius - distance;
	if (penetration > 0)
	{
		a.IsCollided = true;
		b.IsCollided = true;

		double ka = (1 / a.GetMass()) / (1 / a.GetMass() + 1 / b.GetMass());
		double kb = (1 / b.GetMass()) / (1 / a.GetMass() + 1 / b.GetMass());
		deltaPosition.Normalize();
		a.Position.Sub(deltaPosition * (penetration * ka));
		b.Position.Add(deltaPosition * (penetration * kb));
		double deltaVelocity = (b.Velocity - a.Velocity) * deltaPosition - b.GetRadiusChangeSpeed() - a.GetRadiusChangeSpeed();
		if (deltaVelocity < 0)
		{
			Vector3D impulse =
				//(1 + Consants.MIN_HIT_E + Rnd.NextDouble() * (Consants.MAX_HIT_E - Consants.MIN_HIT_E)) *
				deltaPosition * ((1 + hitE) *
				deltaVelocity);
			a.Velocity.Add(impulse * ka);
			b.Velocity.Sub(impulse * kb);
		}
	}
}

std::optional<Vector3D> Simulator::CollideWithArena(Entity & e)
{
	Dan dan = DanCalculator::GetDanToArena(e.Position, Constants::Rules.arena);
	double penetration = e.Radius - dan.Distance;
	if (penetration > 0)
	{
		e.Position.Add(dan.Normal * penetration);
		e.IsArenaCollided = true;
		double velocity = e.Velocity * dan.Normal - e.GetRadiusChangeSpeed();
		if (velocity < 0)
		{
			e.Velocity.Sub(dan.Normal * ((1 + e.GetArenaE()) * velocity));			
			return dan.Normal;
		}
	}

	return std::nullopt;
}

void Simulator::Move(Entity & e, double deltaTime)
{
	e.Velocity = Helper::Clamp(e.Velocity, Constants::Rules.MAX_ENTITY_SPEED);
	e.Position.Add(e.Velocity * deltaTime);
	e.Position.Y -= Constants::Rules.GRAVITY * deltaTime * deltaTime / 2;
	e.Velocity.Y -= Constants::Rules.GRAVITY * deltaTime;
}

PositionVelocityContainer Simulator::GetRobotPVContainer(
	const Vector3D & startPosition, const Vector3D & targetPosition, const Vector3D & startVelocity, 
	int ticks, double velocityCoeff)
{
	double tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
	double microTickTime = tickTime / Constants::Rules.MICROTICKS_PER_TICK;
	auto const tvLength = Constants::Rules.ROBOT_MAX_GROUND_SPEED * velocityCoeff;

	int getDirectionMicroTicks = 0;
	Vector3D velocity = Vector3D(startVelocity);
	Vector3D position = Vector3D(startPosition);
	double distToTarget2 = Helper::GetLength2(position, targetPosition);

	Vector3D targetVelocity =
		Helper::GetTargetVelocity(startPosition, targetPosition, tvLength);


	double velocityPosSign = targetVelocity.X * velocity.Z - targetVelocity.Z * velocity.X;

	bool isGettingCloser = false;
	bool isPassedBy = false;

	const auto tv1Length = Constants::Rules.ROBOT_ACCELERATION * microTickTime;

	//поворачиваем в направлении direction в рамках 1 тика
	while (getDirectionMicroTicks + Constants::Rules.MICROTICKS_PER_TICK <= ticks * Constants::Rules.MICROTICKS_PER_TICK)
	{
		Vector3D tvc1 = Helper::GetTargetVelocity(velocity, targetVelocity, tv1Length);

		Vector3D newVelocity = Vector3D(
			velocity.X + tvc1.X * Constants::Rules.MICROTICKS_PER_TICK,
			velocity.Y + tvc1.Y * Constants::Rules.MICROTICKS_PER_TICK,
			velocity.Z + tvc1.Z * Constants::Rules.MICROTICKS_PER_TICK);

		double newVelocityPosSign = targetVelocity.X * newVelocity.Z - targetVelocity.Z * newVelocity.X;
		if (velocityPosSign * newVelocityPosSign <= 0) break;

		position.Add(velocity * (Constants::Rules.MICROTICKS_PER_TICK * microTickTime) + tvc1 *
			(microTickTime * Constants::Rules.MICROTICKS_PER_TICK * (Constants::Rules.MICROTICKS_PER_TICK + 1) / 2.0));

		double newDistToTarget2 = Helper::GetLength2(position, targetPosition);
		if (!isGettingCloser)
		{
			if (newDistToTarget2 < distToTarget2)
			{
				isGettingCloser = true;
			}
		}
		else if (!isPassedBy)
		{
			if (newDistToTarget2 > distToTarget2) isPassedBy = true;
		}
		distToTarget2 = newDistToTarget2;

		velocity = newVelocity;

		getDirectionMicroTicks += Constants::Rules.MICROTICKS_PER_TICK;

		targetVelocity = Helper::GetTargetVelocity(position, targetPosition, tvLength);
		velocityPosSign = targetVelocity.X * velocity.Z - targetVelocity.Z * velocity.X;
	}

	

	//докручиваем до direction в рамках 1 тика
	Vector3D targetVelocityChange = targetVelocity - velocity;
	double tvcLength2 = targetVelocityChange.Length2();
	if (tvcLength2 > Eps2 && 
		getDirectionMicroTicks < ticks * Constants::Rules.MICROTICKS_PER_TICK &&
		abs(targetVelocity.X * velocity.Z - targetVelocity.Z * velocity.X) > Eps)
	{		
		const auto aVectorLength = Constants::Rules.ROBOT_ACCELERATION * microTickTime;
		const auto n = int(targetVelocityChange.Length() / aVectorLength);

		if (n > 0)
		{
			targetVelocityChange.Normalize();
			Vector3D tvc1 = targetVelocityChange * (Constants::Rules.ROBOT_ACCELERATION * microTickTime);
			
			position.Add(velocity * (n * microTickTime) + tvc1 *
				(microTickTime * n * (n + 1) / 2.0));
			if (isGettingCloser && !isPassedBy)
			{
				if (Helper::GetLength2(position, targetPosition) > distToTarget2) isPassedBy = true;
			}

			velocity = velocity + tvc1 * n;
			getDirectionMicroTicks += n;
		}

		targetVelocityChange = targetVelocity - velocity;		
		tvcLength2 = targetVelocityChange.Length2();

		if (tvcLength2 > Eps2 &&
			getDirectionMicroTicks < ticks * Constants::Rules.MICROTICKS_PER_TICK &&
			abs(targetVelocity.X * velocity.Z - targetVelocity.Z * velocity.X) > Eps)
		{
			targetVelocityChange.Normalize();
			velocity.Add(Helper::Clamp2(
				targetVelocityChange * (Constants::Rules.ROBOT_ACCELERATION * microTickTime),
				tvcLength2));
			position.Add(velocity * microTickTime);		

			getDirectionMicroTicks += 1;
		}
	}

	//остальные микротики тика докручивания движемся прямо
	double addMoveMicroTicks = getDirectionMicroTicks % Constants::Rules.MICROTICKS_PER_TICK == 0
		? 0
		: Constants::Rules.MICROTICKS_PER_TICK - getDirectionMicroTicks % Constants::Rules.MICROTICKS_PER_TICK;

	if (addMoveMicroTicks > 0)
	{
		getDirectionMicroTicks += addMoveMicroTicks;
		Vector3D newPosition = position + velocity * (microTickTime * addMoveMicroTicks);
		if (!isPassedBy)
		{
			isPassedBy = Helper::GetLength2(newPosition, position) > Helper::GetLength2(targetPosition, position);
		}
		position = newPosition;
	}


	if (getDirectionMicroTicks == ticks * Constants::Rules.MICROTICKS_PER_TICK)
		return PositionVelocityContainer(position, velocity, isPassedBy);

	//код дальше справедлив, только если мы за отвеведенное время не доберемся до целевой точки
	Vector3D startRunPosition = Vector3D(position);

	Vector3D tvc2 = Helper::GetTargetVelocity(position, targetPosition, Constants::Rules.ROBOT_ACCELERATION * microTickTime);
	//разгоняемся до максимальной скорости
	double maxVelocityTime = (tvLength - velocity.Length()) / Constants::Rules.ROBOT_ACCELERATION;
	double accelerationMicroTicks = maxVelocityTime * Constants::Rules.TICKS_PER_SECOND *
		Constants::Rules.MICROTICKS_PER_TICK; //TODO: здесь д.б. int

	if (accelerationMicroTicks > ticks * Constants::Rules.MICROTICKS_PER_TICK)
		accelerationMicroTicks = ticks * Constants::Rules.MICROTICKS_PER_TICK;

	position.Add(velocity * (accelerationMicroTicks * microTickTime) + tvc2 *
		(microTickTime * accelerationMicroTicks * (accelerationMicroTicks + 1) / 2.0));
	velocity = Vector3D(
		velocity.X + tvc2.X * accelerationMicroTicks,
		velocity.Y + tvc2.Y * accelerationMicroTicks,
		velocity.Z + tvc2.Z * accelerationMicroTicks);	
		
	double getMaxVelocityMicroTicks = getDirectionMicroTicks + accelerationMicroTicks;


	//движемся оставшееся время с максимальной скоростью
	double resMicroTicks = ticks * Constants::Rules.MICROTICKS_PER_TICK - getMaxVelocityMicroTicks;
	double resTime = resMicroTicks * microTickTime;
	position.Add(velocity * resTime);

	if (!isPassedBy)
	{
		isPassedBy = Helper::GetLength2(position, startRunPosition) >
			Helper::GetLength2(targetPosition, startRunPosition);
	}

	return PositionVelocityContainer(position, velocity, isPassedBy);
}

std::optional<double> Simulator::GetCollisionT(
	const Vector3D & pR, const Vector3D & vR, const Vector3D & pB, const Vector3D & vB, double r1, double r2)
{
	double bDiv2 = (vB.X - vR.X) * (pB.X - pR.X) + (vB.Y - vR.Y) * (pB.Y - pR.Y) +
		(vB.Z - vR.Z) * (pB.Z - pR.Z);
	double a = (vB.X - vR.X) * (vB.X - vR.X) + (vB.Y - vR.Y) * (vB.Y - vR.Y) + (vB.Z - vR.Z) * (vB.Z - vR.Z);
	double c = (pB.X - pR.X) * (pB.X - pR.X) + (pB.Y - pR.Y) * (pB.Y - pR.Y) + (pB.Z - pR.Z) * (pB.Z - pR.Z) -
		(r1 + r2) *
		(r1 + r2);

	double d1 = bDiv2 * bDiv2 - a * c;
	if (d1 < 0) return std::nullopt; // no collision

	double collisionT = (-bDiv2 - sqrt(d1)) / a;
	if (collisionT < 0) return std::nullopt;
	return collisionT;
}

void Simulator::UpdateOnAir(
	BallEntity& entity, std::vector<RobotEntity>& jumpRes, double deltaTime, double hitE, bool & isGoalScored,
	bool simulateArenaCollision)
{
	Move(entity, deltaTime);
	for (auto & re : jumpRes)
	{
		Move(re, deltaTime);

		re.Radius = GetRobotRadius(re.Action.jump_speed);
		re.SetRadiusChangeSpeed(re.Action.jump_speed);
	}


	for (int i = 0; i < jumpRes.size(); ++i)
	{
		for (int j = 0; j < i; ++j)
		{
			CollideEntities(jumpRes[i], jumpRes[j], hitE);
		}
	}

	for (auto & re : jumpRes)
		CollideEntities(re, entity, hitE);

	if (simulateArenaCollision)
	{
		for (auto & re : jumpRes)
		{
			std::optional<Vector3D> collisionNormal = CollideWithArena(re);
			if (collisionNormal == std::nullopt)
				re.Touch = false;
			else
			{
				re.Touch = true;
				re.TouchNormal = *collisionNormal;
			}
		}
		CollideWithArena(entity);
	}


	//if (entity.Radius < Constants::Rules.BALL_RADIUS - Eps) return; //пережитк для робота

	if (abs(entity.Position.Z) > Constants::Rules.arena.depth / 2 + entity.Radius)
		isGoalScored = true;

}

void Simulator::Update(RobotEntity& robot, BallEntity& ball, double deltaTime, double hitE, bool & isGoalScored)
{
	isGoalScored = false;
	
	if (robot.Touch)
	{
		Vector3D targetVelocity = Helper::GetAcionTargetVelocity(robot.Action);
		targetVelocity = Helper::Clamp(targetVelocity, Constants::Rules.ROBOT_MAX_GROUND_SPEED, 
			Constants::Rules.ROBOT_MAX_GROUND_SPEED*Constants::Rules.ROBOT_MAX_GROUND_SPEED);
		targetVelocity.Sub(robot.TouchNormal * (robot.TouchNormal * targetVelocity));
		Vector3D targetVelocityChange = targetVelocity - robot.Velocity;
		double tvcLength2 = targetVelocityChange.Length2();
		if (tvcLength2 > Eps2)
		{
			double acceleration = Constants::Rules.ROBOT_ACCELERATION * std::max(0.0, robot.TouchNormal.Y);			
			targetVelocityChange.Normalize();
			robot.Velocity.Add(Helper::Clamp2 (targetVelocityChange * (acceleration * deltaTime),
				tvcLength2));
		}
	}

	if (robot.Action.use_nitro)
	{
		Vector3D targetVelocityChange = Helper::Clamp(Helper::GetAcionTargetVelocity(robot.Action) - robot.Velocity,
			robot.Nitro * Constants::Rules.NITRO_POINT_VELOCITY_CHANGE);
		double tvcLength = targetVelocityChange.Length();
		if (tvcLength > 0)
		{
			targetVelocityChange.Normalize();
			Vector3D acceleration = targetVelocityChange * Constants::Rules.ROBOT_NITRO_ACCELERATION;
			Vector3D velocityChange = Helper::Clamp(acceleration * deltaTime, tvcLength);
			robot.Velocity.Add(velocityChange);
			robot.Nitro -= velocityChange.Length() / Constants::Rules.NITRO_POINT_VELOCITY_CHANGE;
		}
	}

	Move(robot, deltaTime);

	robot.Radius = GetRobotRadius(robot.Action.jump_speed);
	robot.SetRadiusChangeSpeed(robot.Action.jump_speed);
	

	Move(ball, deltaTime);

	/*for (int i = 0; i < robots.Count(); ++i)
	{
		for (var j = 0; j < i; ++j)
		{
			CollideEntities(robots[i], robots[j], (Consants.MIN_HIT_E + Consants.MAX_HIT_E) / 2d);
		}
	}*/

	
	CollideEntities(robot, ball, hitE);
	std::optional<Vector3D> collisionNormal = CollideWithArena(robot);
	if (collisionNormal == std::nullopt)
		robot.Touch = false;
	else
	{
		robot.Touch = true;
		robot.TouchNormal = *collisionNormal;
	}
	

	CollideWithArena(ball);

	if (abs(ball.Position.Z) > Constants::Rules.arena.depth / 2 + ball.Radius)
		isGoalScored = true;

	/*foreach(var robot in robots)
	{
		if (Math.Abs(robot.Nitro - Consants.MAX_NITRO_AMOUNT) < Eps) continue;
		foreach(var pack in nitroPacks)
		{
			if (pack.Alive) continue;
			if (Helper.GetLength(robot.Position, pack.Position) <= robot.Radius + pack.Radius)
			{
				robot.Nitro = Consants.MAX_NITRO_AMOUNT;
				pack.RespawnTicks = Consants.NITRO_RESPAWN_TICKS;
			}
		}
	}*/
}

void Simulator::Tick(BallEntity& entity, std::vector<RobotEntity>& jumpRes, double hitE, bool& isGoalScored)
{
	double deltaTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
	isGoalScored = false;
	for (int i = 0; i < Constants::Rules.MICROTICKS_PER_TICK; ++i)
	{
		UpdateOnAir(entity, jumpRes, deltaTime / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored, true);
	}
}

void Simulator::Tick(RobotEntity& robot, BallEntity ball)
{
	double deltaTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
	bool isGoalScored = false;
	for (int i = 0; i < Constants::Rules.MICROTICKS_PER_TICK; ++i)
	{
		Update(robot, ball, deltaTime / Constants::Rules.MICROTICKS_PER_TICK,
			(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
	}

	/*foreach(NitroPackContainer pack in nitroPacks)
	{
		if (pack.Alive) continue;
		pack.RespawnTicks -= 1;
	}*/
}

void Simulator::simulate_jump_start(RobotEntity& re)
{

	//set velocity
	Vector3D targetVelocity = Helper::GetAcionTargetVelocity(re.Action);	
	Vector3D targetVelocityChange = targetVelocity - re.Velocity;
	double tvcLength2 = targetVelocityChange.Length2();
	const double mtTime = 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;
	const double a = Constants::Rules.ROBOT_ACCELERATION;

	if (tvcLength2 > Eps2)
	{
		targetVelocityChange.Normalize();
		re.Velocity.Add(Helper::Clamp2(targetVelocityChange * (a * mtTime),
			tvcLength2));
	}

	//update pos
	re.Position.X += re.Velocity.X * mtTime;
	re.Position.Z += re.Velocity.Z * mtTime;

	//update radius
	re.Radius = GetRobotRadius(re.Action.jump_speed);
	re.SetRadiusChangeSpeed(re.Action.jump_speed);

	//collide with arena
	double penetration = Constants::Rules.ROBOT_MAX_RADIUS - Constants::Rules.ROBOT_MIN_RADIUS;

	re.Position.Y += penetration;
	re.IsArenaCollided = true;
	Vector3D danNormal = Vector3D(0, 1, 0);
	double velocity = -re.GetRadiusChangeSpeed();
	if (velocity < 0)
	{
		re.Velocity.Sub(danNormal * ((1 + re.GetArenaE()) * velocity));
	}

	
	//2nd tick set velocity
	targetVelocityChange = targetVelocity - re.Velocity;
	tvcLength2 = targetVelocityChange.Length2();
	if (tvcLength2 > Eps2)
	{
		targetVelocityChange.Normalize();
		re.Velocity.Add(Helper::Clamp2(targetVelocityChange * (a * mtTime),
			tvcLength2));
	}

	//update pos
	re.Position.X += re.Velocity.X * mtTime;
	re.Position.Y += re.Velocity.Y * mtTime - Constants::Rules.GRAVITY * mtTime * mtTime / 2.0;
	re.Position.Z += re.Velocity.Z * mtTime;

	//update v_y
	re.Velocity.Y -= Constants::Rules.GRAVITY * mtTime;

	re.Touch = false;

}
