#include "MyStrategy.h"
#include "Simulator.h"
#include "Helper.h"
#include <cmath>

using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) 
{ 
	if (game.current_tick == 0)
	{
		Constants::Rules = rules;
		Init(rules);
		DanCalculator::Init(rules.arena);
		
		_oppGates = Vector3D(0, 0, rules.arena.depth / 2 + rules.arena.goal_side_radius);
		_oppGatesLeftCorner = Vector3D(-rules.arena.goal_width / 2,
			0,
			rules.arena.depth / 2 + rules.arena.goal_side_radius);
		_oppGatesRightCorner = Vector3D(rules.arena.goal_width / 2,
			0,
			rules.arena.depth / 2 + rules.arena.goal_side_radius);

		_myGatesLeftCorner = Vector3D(-rules.arena.goal_width / 2,
			0,
			-rules.arena.depth / 2 + rules.arena.goal_side_radius);
		_myGatesRightCorner = Vector3D(rules.arena.goal_width / 2,
			0,
			-rules.arena.depth / 2 + rules.arena.goal_side_radius);

		_myGates = Vector3D(0, 0, -rules.arena.depth / 2 - rules.arena.goal_side_radius);
		_beforeMyGates = Vector3D(0, 0, -rules.arena.depth / 2 + rules.arena.corner_radius / 2);
		_distToFollowBall = rules.arena.depth / 3;


		_penaltyAreaZ = -rules.arena.depth / 4;
		_penaltyMinX = -rules.arena.goal_width / 4 - (rules.arena.width / 2 - rules.arena.goal_width / 2) / 2;
		_penaltyMaxX = rules.arena.goal_width / 4 + (rules.arena.width / 2 - rules.arena.goal_width / 2) / 2;

	}

	_isFirstRobot = !_isFirstRobot;
	if (_isFirstRobot)
	{
		_ballEntities = std::map<int, BallEntity>();
		_actions = std::map<int, model::Action > ();
	}

	if (abs(game.ball.z) > rules.arena.depth / 2 + game.ball.radius) //goal scored
	{
		_defenderId = -1;
		return;
	}

	if (!_isFirstRobot)
	{
		InitAction(action, me.id);
		return;
	}


	std::map<int, std::optional<double>> collisionTimes = std::map<int, std::optional<double>>();
	auto const ballEntity = BallEntity(game.ball);
	auto const isMeGoalPossible = IsGoalBallDirection2(ballEntity, -1);
	
	for(Robot robot : game.robots)
	{
		if (!robot.is_teammate) continue;

		Action robotAction;
		std::optional<double> collisionT = std::nullopt;
		if (!robot.touch)
		{
			robotAction = model::Action();

			RobotEntity re = RobotEntity(robot);
			model::Action reAction = Action();
			reAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
			re.Action = reAction;

			Simulator::Tick(re, BallEntity(game.ball));
			if (!re.IsArenaCollided) robotAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
			_actions[robot.id] = robotAction;
			collisionT = Simulator::GetCollisionT(
				Helper::GetRobotPosition(robot),
				Helper::GetRobotVelocity(robot),
				Helper::GetBallPosition(game.ball),
				Helper::GetBallVelocity(game.ball));
			collisionTimes[robot.id] = collisionT;
			continue;
		}

		bool meIsDefender = false;
		if (_defenderId >= 0)
		{
			if (robot.id == _defenderId) meIsDefender = true;
		}
		else
		{
			bool hasCloser = false;
			for (Robot r : game.robots)
			{
				if (!r.is_teammate) continue;
				if (r.id == robot.id) continue;
				if (r.z < robot.z) hasCloser = true;
			}
			if (!hasCloser)
			{
				meIsDefender = true;
				_defenderId = robot.id;
			}
		}


		if (!meIsDefender)
		{
			robotAction = SetAttackerAction(robot, game.ball, isMeGoalPossible, collisionT);
		}
		else //Defender
		{
			robotAction = SetDefenderAction(robot, game.ball, _myGates, isMeGoalPossible, collisionT);
		}
		collisionTimes[robot.id] = collisionT;
		_actions[robot.id] = robotAction;
	}

	for(Robot robot: game.robots)
	{
		if (!robot.is_teammate) continue;
		if (!robot.touch) continue;

		Robot otherRobot{};
		for (Robot r : game.robots)
		{
			if (!r.is_teammate || r.id == robot.id) continue;
			otherRobot = r;
			break;
		}

		if (collisionTimes[robot.id] != std::nullopt && collisionTimes[otherRobot.id] != std::nullopt)
		{
			if (*collisionTimes[robot.id] > *collisionTimes[otherRobot.id])
			{
				_actions[robot.id] =
					GetDefaultAction(robot, _defenderId == robot.id ? _myGates : _beforeMyGates);
			}
		}
	}

	InitAction(action, me.id);
}

void MyStrategy::Init(const model::Rules & rules)
{
	_maxStrikeDist =
		rules.BALL_RADIUS + rules.ROBOT_MIN_RADIUS + rules.ROBOT_MIN_RADIUS * 3;
	_maxStrikeDist2 = _maxStrikeDist * _maxStrikeDist;

	_maxDefenderStrikeDist = rules.BALL_RADIUS + rules.ROBOT_MIN_RADIUS + rules.ROBOT_MIN_RADIUS * 4; //TODO
	_maxDefenderStrikeDist2 = _maxDefenderStrikeDist * _maxDefenderStrikeDist;
	_moveSphereRadius = rules.BALL_RADIUS + rules.ROBOT_MIN_RADIUS + rules.ROBOT_MIN_RADIUS * 4; //TODO

}

void MyStrategy::InitAction(model::Action& action, int id)
{
	action.jump_speed = _actions[id].jump_speed;
	action.target_velocity_x = _actions[id].target_velocity_x;
	action.target_velocity_y = _actions[id].target_velocity_y;
	action.target_velocity_z = _actions[id].target_velocity_z;
	action.use_nitro = _actions[id].use_nitro;
}

model::Action MyStrategy::GetDefaultAction(const model::Robot & me, const Vector3D & defaultPos)
{
	Vector3D targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, defaultPos.X, 0, defaultPos.Z,
		Constants::Rules.ROBOT_MAX_GROUND_SPEED);

	model::Action action = model::Action();
	action.target_velocity_x = targetVelocity.X;
	action.target_velocity_y = 0.0;
	action.target_velocity_z = targetVelocity.Z;
	action.jump_speed = 0.0;
	action.use_nitro = false;
	return action;
}

BallEntity MyStrategy::SimulateTickBall(const BallEntity & ballEntity, bool & isGoalScored)
{
	BallEntity bec = BallEntity(ballEntity);
	Simulator::Update(bec, 1.0 / Constants::Rules.TICKS_PER_SECOND, isGoalScored);
	if (!bec.IsArenaCollided) return bec;

	bec = BallEntity(ballEntity);
	Simulator::Tick(bec);
	bec.IsArenaCollided = false;
	return bec;
}

bool MyStrategy::IsPenaltyArea(const Vector3D & position)
{
	return position.Z <= _penaltyAreaZ;
}

double MyStrategy::GetVectorAngleToHorizontal(const Vector3D & v)
{
	double horVector =
		sqrt(v.X*v.X + v.Z*v.Z);
	double angle = atan(v.Y / horVector);
	return angle;
}

int MyStrategy::CompareBallVelocities(const Vector3D & v1, const std::optional<Vector3D>& v2)
{
	if (v2 == std::nullopt) return -1;
	if (v1.Z * v2.value().Z < 0)
	{
		return v1.Z > 0 ? -1 : 1;
	}

	double v1HorAngle = GetVectorAngleToHorizontal(v1);
	double v2HorAngle = GetVectorAngleToHorizontal(v2.value());
	if (v1HorAngle > M_PI / 6 && v2HorAngle > M_PI / 6)
		return abs(v1.Z) > abs(v2.value().Z) ? -1 : 1;
	return v1HorAngle > v2HorAngle ? -1 : 1;
}

bool MyStrategy::IsGoalBallDirection2(const BallEntity & startBallEntity, int directionCoeff)
{
	if (directionCoeff == -1)
	{
		if (startBallEntity.Position.Z < -Constants::Rules.arena.depth / 2 + Constants::Rules.arena.depth / 8 &&
			abs(startBallEntity.Position.X) < Constants::Rules.arena.goal_width / 2 + 2.5)
			return true;
	}

	if (startBallEntity.Velocity.Z * directionCoeff <= 0) return false;

	BallEntity ballEntity = BallEntity(startBallEntity);
	while (true)
	{
		double yTime = (ballEntity.Velocity.Y +
			sqrt(ballEntity.Velocity.Y * ballEntity.Velocity.Y -
				2 * Constants::Rules.GRAVITY * (Constants::Rules.BALL_RADIUS - ballEntity.Position.Y))) /
			Constants::Rules.GRAVITY;
		double zTime = ((Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS) * directionCoeff - ballEntity.Position.Z) / ballEntity.Velocity.Z;
		if (yTime < zTime)
		{
			double ticks =
				trunc(yTime * Constants::Rules.TICKS_PER_SECOND * Constants::Rules.MICROTICKS_PER_TICK);
			double time = ticks * 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;

			Simulator::Move(ballEntity, time);//доходим до коллизии с ареной
			bool isGoalScored = false;
			Simulator::Update(ballEntity, 
				1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
				isGoalScored);

			if (abs(ballEntity.Position.X) > Constants::Rules.arena.goal_width / 2) return false;
		}
		else break;//TODO: возможно, стоит рассчитать положение в момент zTime
	}

	bool isGoalScored = false;
	bool isZIncreasing = true;
	double z = ballEntity.Position.Z;
	while (!isGoalScored && isZIncreasing)
	{
		ballEntity = SimulateTickBall(ballEntity, isGoalScored);
		isZIncreasing = ballEntity.Position.Z * directionCoeff > z * directionCoeff;
		z = ballEntity.Position.Z;
	}

	return isGoalScored;
}

model::Action MyStrategy::SetDefenderAction(
	const model::Robot & me, const model::Ball & ball, const Vector3D & defenderPoint,
	bool isMeGoalPossible,
	std::optional<double>& collisionT)
{
	model::Action action = model::Action();
	Vector3D targetVelocity;

	std::optional<Vector3D> bestBallVelocity = std::nullopt;
	std::optional<Vector3D> defendPoint = 
		GetDefenderMovePoint(me, ball, isMeGoalPossible, collisionT, bestBallVelocity);

	std::optional<double> jumpCollisionT = std::nullopt;
	std::optional<Vector3D> jumpBallVelocity = std::nullopt;

	if (IsOkDefenderPosToJump(
		Helper::GetRobotPosition(me),
		Helper::GetRobotVelocity(me),
		Helper::GetBallPosition(ball),
		Helper::GetBallVelocity(ball),
		isMeGoalPossible,
		jumpCollisionT,
		jumpBallVelocity))
	{
		if (defendPoint == std::nullopt || CompareBallVelocities(*jumpBallVelocity, bestBallVelocity) < 0)
		{
			collisionT = jumpCollisionT;

			targetVelocity =
				Helper::GetTargetVelocity(me.x, 0, me.z, ball.x, 0, ball.z, Constants::Rules.ROBOT_MAX_GROUND_SPEED);
			action.target_velocity_x = targetVelocity.X;
			action.target_velocity_y = 0.0;
			action.target_velocity_z = targetVelocity.Z;
			action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
			action.use_nitro = false;

			return action;
		}
	}


	if (defendPoint == std::nullopt) {
		defendPoint = defenderPoint;
	}

	targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, defendPoint.value().X, 0, defendPoint.value().Z,
		Constants::Rules.ROBOT_MAX_GROUND_SPEED);

	action.target_velocity_x = targetVelocity.X;
	action.target_velocity_y = 0.0;
	action.target_velocity_z = targetVelocity.Z;
	action.jump_speed = 0.0;
	action.use_nitro = false;
	return action;
}

std::optional<Vector3D> MyStrategy::GetDefenderStrikeBallVelocity(const model::Robot & robot, int t,
	bool isMeGoalPossible,
	std::optional<double>& collisionT, bool & isPassedBy)
{
	std::optional<Vector3D> bestBallVelocity = std::nullopt;
	isPassedBy = false;
	BallEntity ballEntity = _ballEntities[t];

	if (!IsPenaltyArea(ballEntity.Position))
	{
		return std::nullopt;
	}

	Vector3D targetPos = Vector3D(ballEntity.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
		ballEntity.Position.Z);

	for (int moveT = 1; moveT < t; ++moveT)
	{
		PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
			Helper::GetRobotPosition(robot),
			targetPos,
			Helper::GetRobotVelocity(robot),
			moveT);

		if (pvContainer.IsPassedBy)
		{
			isPassedBy = true;
			return bestBallVelocity; // проскочим целевую точку. дальше все непредсказуемо
		}

		if (_ballEntities[moveT].Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)//м§ч в воротах
			return bestBallVelocity;

		std::optional<double> curCollisionT = std::nullopt;
		std::optional<Vector3D> collisionBallVelocity = std::nullopt;

		if (IsOkDefenderPosToJump(pvContainer.Position, pvContainer.Velocity,
			_ballEntities[moveT].Position, _ballEntities[moveT].Velocity,
			isMeGoalPossible,
			curCollisionT, collisionBallVelocity))
		{
			if (CompareBallVelocities(*collisionBallVelocity, bestBallVelocity) < 0)
			{
				bestBallVelocity = collisionBallVelocity;
				collisionT = curCollisionT;
			}
		}
	}

	return bestBallVelocity;
}

bool MyStrategy::IsOkDefenderPosToJump(
	const Vector3D & robotPosition, const Vector3D & robotVelocity,  
	const Vector3D & moveTBePosition,const Vector3D & moveTBeVelocity,
	bool isMeGoalPossible,
	std::optional<double>& collisionT, std::optional<Vector3D>& collisionBallVelocity)
{
	collisionBallVelocity = std::nullopt;

	Vector3D targetVelocity =
		Helper::GetTargetVelocity(robotPosition.X, 0, robotPosition.Z, moveTBePosition.X, 0, moveTBePosition.Z,
			Constants::Rules.ROBOT_MAX_GROUND_SPEED);

	RobotEntity re = RobotEntity(
		Vector3D(robotPosition), Vector3D(robotVelocity), Constants::Rules.ROBOT_MIN_RADIUS,
		true, Vector3D(0, 1, 0), 0.0);
	model::Action action = model::Action();
	action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
	action.target_velocity_x = targetVelocity.X;
	action.target_velocity_y = 0.0;
	action.target_velocity_z = targetVelocity.Z;
	action.use_nitro = false;
	re.Action = action;

	BallEntity moveTBallEntity = BallEntity(Vector3D(moveTBePosition), Vector3D(moveTBeVelocity));


	//симулируем отрыв от земли
	bool isGoalScored = false;
	Simulator::Update(re, moveTBallEntity, 
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
	Simulator::Update(re, moveTBallEntity, 
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);

	if (moveTBallEntity.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
	{
		return false;
	}

	re.IsArenaCollided = false;

	collisionT = Simulator::GetCollisionT(re.Position, re.Velocity, moveTBallEntity.Position,
		moveTBallEntity.Velocity);
	if (collisionT == std::nullopt) return false;

	//var leftT = (t - moveT) * Consants.TICKS_PER_SECOND;
	//if (collisionT > leftT) continue;

	double microTicks =
		trunc(*collisionT * Constants::Rules.TICKS_PER_SECOND * Constants::Rules.MICROTICKS_PER_TICK);
	//ситуаци§ за микротик до коллизии.
	//TODO: Неточно, т.к. может быть коллизи§ м§ча с ареной или коллизи§ робота с ареной
	Simulator::Update(re, moveTBallEntity, 
		microTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);

	if (moveTBallEntity.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
		return false;

	if (re.Velocity.Y < 0) return false;//не бьем на излете
	if (re.Position.Y >= moveTBallEntity.Position.Y) return false;//не бьем сверху-вниз
	if (re.IsArenaCollided) return false;

	//коллизи§ вне штрафной площади
	if (!IsPenaltyArea(moveTBallEntity.Position))
		return false;

	double hitE = (Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0;

	Simulator::Update(re, moveTBallEntity, 
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored);

	if (moveTBallEntity.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
		return false;

	if (!moveTBallEntity.IsCollided) //веро§тно, м§ч ударилс§ об арену. прыгать смысла нет
	{
		return false;
	}

	collisionBallVelocity = moveTBallEntity.Velocity;

	if (moveTBallEntity.Velocity.Z >= 0) //выбиваем от ворот и вверх. TODO
		return true;

	if (!isMeGoalPossible) return false;
	const auto isCollisionGoalPossible = IsGoalBallDirection2(moveTBallEntity, -1);
	
	return !isCollisionGoalPossible;
}

std::optional<Vector3D> MyStrategy::GetDefenderMovePoint(const model::Robot & robot, const model::Ball & ball,
	bool isMeGoalPossible,
	std::optional<double>& collisionT, std::optional<Vector3D>& bestBallVelocity)
{
	BallEntity ballEntity = BallEntity(ball);
	bestBallVelocity = std::nullopt;
	std::optional<Vector3D> movePoint = std::nullopt;

	for (int t = 1; t <= BallMoveTicks; ++t)
	{
		if (_ballEntities.count(t) > 0) ballEntity = _ballEntities[t];
		else
		{
			bool isGoalScored = false;
			ballEntity = SimulateTickBall(ballEntity, isGoalScored);
			_ballEntities[t]= ballEntity;
		}

		std::optional<double> jumpCollisionT = std::nullopt;
		bool isPassedBy = false;
		std::optional<Vector3D> ballVelocity = 
			GetDefenderStrikeBallVelocity(robot, t, isMeGoalPossible, jumpCollisionT, isPassedBy);
		if (isPassedBy)
			return movePoint;

		if (ballVelocity == std::nullopt) continue;
		if (CompareBallVelocities(*ballVelocity, bestBallVelocity) < 0)
		{
			bestBallVelocity = ballVelocity;
			collisionT = t * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();

			movePoint = Vector3D(_ballEntities[t].Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
				_ballEntities[t].Position.Z);
		}
	}
	return movePoint;
}

model::Action MyStrategy::SetAttackerAction(const model::Robot & me, const model::Ball & ball,
	bool isMeGoalPossible,
	std::optional<double>& collisionT)
{
	model::Action action = model::Action();

	Vector3D targetVelocity;
	RobotEntity robotEntity = RobotEntity(me);

	if (IsOkPosToJump(BallEntity(ball), robotEntity, collisionT))
	{
		targetVelocity =
			Helper::GetTargetVelocity(me.x, 0, me.z, ball.x, 0, ball.z, Constants::Rules.ROBOT_MAX_GROUND_SPEED);
		action.target_velocity_x = targetVelocity.X;
		action.target_velocity_y = 0.0;
		action.target_velocity_z = targetVelocity.Z;
		action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
		action.use_nitro = false;
		return action;
	}

	bool isDefender = false;
	std::optional<Vector3D> bestBallVelocity = std::nullopt;
	std::optional<Vector3D> movePoint = 
		GetAttackerMovePoint(me, ball,isMeGoalPossible, collisionT, isDefender, bestBallVelocity);
	if (isDefender)
	{
		std::optional<double> jumpCollisionT = std::nullopt;
		std::optional<Vector3D> jumpBallVelocity = std::nullopt;

		auto const ballEntity = BallEntity(ball);
		auto const isGoalPossible = IsGoalBallDirection2(ballEntity, -1);
		if (IsOkDefenderPosToJump(
			Helper::GetRobotPosition(me),
			Helper::GetRobotVelocity(me),			
			Helper::GetBallPosition(ball),
			Helper::GetBallVelocity(ball),
			isGoalPossible,
			jumpCollisionT, jumpBallVelocity))
		{
			if (movePoint == std::nullopt || CompareBallVelocities(*jumpBallVelocity, bestBallVelocity) < 0)
			{
				collisionT = jumpCollisionT;

				targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, ball.x, 0, ball.z,
					Constants::Rules.ROBOT_MAX_GROUND_SPEED);
				action.target_velocity_x = targetVelocity.X;
				action.target_velocity_y = 0.0;
				action.target_velocity_z = targetVelocity.Z;
				action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
				action.use_nitro = false;

				return action;
			}
		}

		if (movePoint == std::nullopt) {
			movePoint = _beforeMyGates;
		}

		targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, movePoint.value().X, 0, movePoint.value().Z,
			Constants::Rules.ROBOT_MAX_GROUND_SPEED);

		action.target_velocity_x = targetVelocity.X;
		action.target_velocity_y = 0.0;
		action.target_velocity_z = targetVelocity.Z;
		action.jump_speed = 0.0;
		action.use_nitro = false;
		return action;
	}
	else //действуем как нападающий
	{

		if (movePoint == std::nullopt)
		{
			collisionT = std::nullopt;
			if (Helper::GetLength2(Helper::GetBallPosition(ball), Helper::GetRobotPosition(me)) >
				_distToFollowBall * _distToFollowBall
				&&	me.z < ball.z)
			{
				movePoint = Vector3D(ball.x, 0, ball.z);
			}
			else
			{
				movePoint = _beforeMyGates;
			}
		}

		targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, (*movePoint).X, 0, (*movePoint).Z,
			Constants::Rules.ROBOT_MAX_GROUND_SPEED);

		action.target_velocity_x = targetVelocity.X;
		action.target_velocity_y = 0.0;
		action.target_velocity_z = targetVelocity.Z;
		action.jump_speed = 0.0;
		action.use_nitro = false;
		return action;
	}
}

bool MyStrategy::IsOkPosToMove(const Vector3D & mePos, const model::Robot & robot, const BallEntity & ballEntity, 
	int t, std::optional<double>& collisionT)
{
	collisionT = std::nullopt;
	PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
		Helper::GetRobotPosition(robot),
		mePos,
		Helper::GetRobotVelocity(robot),
		t);

	if (pvContainer.IsPassedBy)
		return false; // проскочим целевую точку. дальше все непредсказуемо

	//TODO: учесть пределы арены по-умному
	if (abs(pvContainer.Position.X) + Constants::Rules.ROBOT_MIN_RADIUS > Constants::Rules.arena.width / 2) 
		return false;
	if (abs(pvContainer.Position.Z) + Constants::Rules.ROBOT_MIN_RADIUS > Constants::Rules.arena.depth / 2)
		if (abs(pvContainer.Position.X) + Constants::Rules.ROBOT_MIN_RADIUS > Constants::Rules.arena.goal_width / 2 ||
			abs(pvContainer.Position.Z) + Constants::Rules.ROBOT_MIN_RADIUS > Constants::Rules.arena.depth / 2 + Constants::Rules.arena.goal_depth) 
			return false;

	if (Helper::GetLength(ballEntity.Position, pvContainer.Position) < 
		Constants::Rules.ROBOT_MIN_RADIUS + Constants::Rules.BALL_RADIUS)
		return false; //столкнемс§ еще при движении в точку

	RobotEntity robotE = RobotEntity(
		Vector3D(pvContainer.Position),
		Vector3D(pvContainer.Velocity),
		Constants::Rules.ROBOT_MIN_RADIUS,
		true,
		Vector3D(0, 1, 0),
		0);

	bool isOkPosToStrike = IsOkPosToJump(BallEntity(ballEntity), robotE, collisionT);
	if (isOkPosToStrike)
	{
		return true;
	}

	return false;
}

std::optional<Vector3D> MyStrategy::GetAttackerMovePoint(const model::Robot & robot, const model::Ball & ball,
	bool isMeGoalPossible,
	std::optional<double>& collisionT, bool & isDefender, std::optional<Vector3D>& bestBallVelocity)
{
	BallEntity ballEntity = BallEntity(ball);
	bestBallVelocity = std::nullopt;
	std::optional<Vector3D> movePoint = std::nullopt;
	isDefender = false;

	for (int t = 1; t <= BallMoveTicks; ++t)
	{
		if (_ballEntities.count(t) > 0) ballEntity = _ballEntities[t];
		else
		{
			bool isGoalScored;
			ballEntity = SimulateTickBall(BallEntity(ballEntity), isGoalScored);
			_ballEntities[t]= ballEntity;
		}

		if (IsPenaltyArea(ballEntity.Position))//включаем режим защитника
		{
			isDefender = true;
			std::optional<double> jumpCollisionT = std::nullopt;
			bool isPassedBy = false;
			std::optional<Vector3D> ballVelocity = 
				GetDefenderStrikeBallVelocity(robot, t, isMeGoalPossible, jumpCollisionT, isPassedBy);
			if (isPassedBy)
				return movePoint;

			if (ballVelocity == std::nullopt) continue;
			if (CompareBallVelocities(ballVelocity.value(), bestBallVelocity) < 0)
			{
				bestBallVelocity = ballVelocity;
				collisionT = t * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();

				movePoint = Vector3D(_ballEntities[t].Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
					_ballEntities[t].Position.Z);
			}
		}
		else
		{
			std::optional<double> jumpCollisionT = std::nullopt;
			movePoint = GetAttackerStrikePoint(robot, t, jumpCollisionT);
			if (movePoint != std::nullopt)
			{
				if (jumpCollisionT == std::nullopt) throw "jumpCollisionT is null";
				collisionT = t * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();
				bestBallVelocity = std::nullopt;
				isDefender = false;
				return movePoint;
			}
		}
	}

	return movePoint;
}

std::optional<Vector3D> MyStrategy::GetAttackerStrikePoint(const model::Robot & robot, int t, 
	std::optional<double>& collisionT)
{
	BallEntity ballEntity = _ballEntities[t];
	if (_beforeStrikePoints.count(robot.id) > 0)
	{
		if (IsOkPosToMove(_beforeStrikePoints[robot.id], robot, ballEntity, t, collisionT))
			return _beforeStrikePoints[robot.id];
		_beforeStrikePoints.erase(robot.id);
	}

	//код, чтобы сразу отбросить лишние варианты. TODO: оптимизировать
	double robotBallDist = sqrt((ballEntity.Position.X - robot.x)* (ballEntity.Position.X - robot.x) +
		(ballEntity.Position.Z - robot.z)* (ballEntity.Position.Z - robot.z));
	double time = t * 1.0 / Constants::Rules.TICKS_PER_SECOND;
	double maxRobotDist = Constants::Rules.ROBOT_MAX_GROUND_SPEED * time +
		Constants::Rules.ROBOT_ACCELERATION * time * time / 2;
	if (robotBallDist > maxRobotDist + _maxStrikeDist)
	{
		collisionT = std::nullopt;
		return std::nullopt;
	}

	double div = (ballEntity.Position.X - robot.x) / (ballEntity.Position.Z - robot.z);
	double tmp = _moveSphereRadius / sqrt(1 + div * div);
	double x1 = ballEntity.Position.X - tmp;
	double z1 = ballEntity.Position.Z - (robot.x - ballEntity.Position.X) * (x1 - ballEntity.Position.X) /
		(robot.z - ballEntity.Position.Z);
	Vector3D rVector1 = Vector3D(x1 - ballEntity.Position.X, 0.0, z1 - ballEntity.Position.Z);

	double x2 = ballEntity.Position.X + tmp;
	double z2 = ballEntity.Position.Z - (robot.x - ballEntity.Position.X) * (x2 - ballEntity.Position.X) /
		(robot.z - ballEntity.Position.Z);
	Vector3D rVector2 = Vector3D(x2 - ballEntity.Position.X, 0.0, z2 - ballEntity.Position.Z);


	Vector3D mePos = Vector3D(ballEntity.Position.X, Constants::Rules.ROBOT_MIN_RADIUS, ballEntity.Position.Z);
	if (IsOkPosToMove(mePos, robot, ballEntity, t, collisionT))
	{
		_beforeStrikePoints[robot.id] = mePos;
		return mePos;
	}

	for (int i = 1; i <= StrikeSphereStepsCount; ++i)
	{
		Vector3D rVectorMult1 = rVector1 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos1 = Vector3D(ballEntity.Position.X + rVectorMult1.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult1.Z);
		if (IsOkPosToMove(mePos1, robot, ballEntity, t, collisionT))
		{
			_beforeStrikePoints[robot.id] = mePos1;
			return mePos1;
		}

		Vector3D rVectorMult2 = rVector2 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos2 = Vector3D(ballEntity.Position.X + rVectorMult2.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult2.Z);
		if (IsOkPosToMove(mePos2, robot, ballEntity, t, collisionT))
		{
			_beforeStrikePoints[robot.id] = mePos2;
			return mePos2;
		}
	}

	return std::nullopt;
}

bool MyStrategy::IsOkPosToJump(BallEntity ballEntity, RobotEntity & robotEntity, std::optional<double>& collisionT)
{
	if (Helper::GetLength2(robotEntity.Position, ballEntity.Position) > _maxStrikeDist2 + EPS)
	{
		collisionT = std::nullopt;
		return false;
	}

	Vector3D targetVelocity = Helper::GetTargetVelocity(robotEntity.Position.X, 0, robotEntity.Position.Z,
		ballEntity.Position.X, 0, ballEntity.Position.Z,
		Constants::Rules.ROBOT_MAX_GROUND_SPEED);

	model::Action action = model::Action();
	action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
	action.target_velocity_x = targetVelocity.X;
	action.target_velocity_y = 0.0;
	action.target_velocity_z = targetVelocity.Z;
	action.use_nitro = false;

	robotEntity.Action = action;

	BallEntity prevBallEntity = BallEntity(ballEntity);

	//симулируем отрыв от земли
	bool isGoalScored = false;
	Simulator::Update(robotEntity, ballEntity, 
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
	Simulator::Update(robotEntity, ballEntity, 
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);

	collisionT = Simulator::GetCollisionT(robotEntity.Position, robotEntity.Velocity, ballEntity.Position,
		ballEntity.Velocity);
	if (collisionT == std::nullopt) return false;

	double microTicks =
		trunc(*collisionT * Constants::Rules.TICKS_PER_SECOND * Constants::Rules.MICROTICKS_PER_TICK);
	//ситуаци§ за микротик до коллизии. Неточно, т.к. может быть коллизи§ м§ча с ареной
	Simulator::Update(robotEntity, ballEntity, 
		microTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);

	//var ticks = microTicks / Consants.MICROTICKS_PER_TICK;
	//for (var i = 0; i < ticks; ++i)
	//{
	//    Simulator.Update(robots, ballEntity, _emptyNitroPacks, 1d / Consants.TICKS_PER_SECOND, out _);
	//}

	//for (var i = 0; i < microTicks - ticks * Consants.MICROTICKS_PER_TICK; ++i)
	//    Simulator.Update(robots, ballEntity, _emptyNitroPacks,
	//        1d / Consants.TICKS_PER_SECOND / Consants.MICROTICKS_PER_TICK, out _);

	//симулируем коллизию
	
	std::vector<double> hitEs = std::vector<double>();
	hitEs.push_back(Constants::Rules.MIN_HIT_E);
	hitEs.push_back(Constants::Rules.MAX_HIT_E);

	for(double hitE: hitEs)
	{
		RobotEntity re2 = RobotEntity(robotEntity);
		BallEntity ballEntity2 = BallEntity(ballEntity);
		Simulator::Update(re2, ballEntity2, 
			1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored);

		if (!ballEntity2.IsCollided) //веро§тно, м§ч ударилс§ об арену. прыгать смысла нет
		{
			//if (!ballEntity.IsArenaCollided)
			//{

			//}

			return false;
		}


		double angle = GetVectorAngleToHorizontal(ballEntity2.Velocity);
		if (angle >M_PI / 3) return false; //не бьем под большим углом к горизонтали

		if (!IsGoalBallDirection2(ballEntity2, 1)) return false;
		double velocityZ = ballEntity2.Velocity.Z;
		if (velocityZ < prevBallEntity.Velocity.Z && IsGoalBallDirection2(prevBallEntity, 1)) return false;

	}

	return true;
}
