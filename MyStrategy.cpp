#include "MyStrategy.h"
#include "Simulator.h"
#include "Helper.h"
#include <cmath>
#include <sstream>
#include <algorithm>

using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) 
{ 
	if (game.current_tick == 0)
	{
		Constants::Rules = rules;
		Init(rules);
		DanCalculator::Init(rules.arena);
	}	

	_isFirstRobot = !_isFirstRobot;

	if (abs(game.ball.z) > rules.arena.depth / 2 + game.ball.radius) //goal scored
	{
		return;
	}

	if (_isFirstRobot)
	{
		InitBallEntities(game.ball);
		_actions = std::map<int, model::Action > ();

		std::vector<Robot> opp_robots = std::vector<Robot>();
		for (Robot robot : game.robots)
		{
			if (!robot.is_teammate) opp_robots.push_back(robot);
		}

		_oppStrikeTime = GetOppStrikeTime(game.ball, opp_robots);
		_goalScoringTick = -1;
	}	

	if (!_isFirstRobot)
	{
		InitAction(action, me.id);
		return;
	}
	_drawSpheres = std::vector<Sphere>();

	std::map<int, std::optional<double>> collisionTimes = std::map<int, std::optional<double>>();
	std::map<int, Vector3D> bestBallVelocities = std::map<int, Vector3D>();//TODO:если робот в воздухе - тут фигня
	auto const ballEntity = BallEntity(game.ball);
	auto const isMeGoalPossible = IsGoalBallDirection2(ballEntity, -1);

	std::vector <Robot> myRobots = std::vector<Robot>();
	Robot defender{};
	std::vector<Robot> attackes = std::vector<Robot>();
	bool isJumping = false;


	for (Robot robot : game.robots)
	{
		if (!robot.is_teammate) continue;
		myRobots.push_back(robot);
		if (!robot.touch)
		{
			isJumping = true;
			InitJumpingRobotAction(robot, game.ball, collisionTimes, bestBallVelocities);
		}

		bool meIsDefender = true;
		for (Robot r : game.robots)
		{
			if (!r.is_teammate) continue;
			if (r.id == robot.id) continue;
			if (r.z < robot.z) meIsDefender = false;
		}
		if (meIsDefender)
			defender = robot;
		else
			attackes.push_back(robot);
	}

	if (isJumping)
	{
		bool isCollisionTime = false;
		int bestCollisionTimeId = -1;
		
		for (auto item:collisionTimes)
		{
			if (item.second != std::nullopt)
			{
				isCollisionTime = true;
				if (bestCollisionTimeId == -1) bestCollisionTimeId = item.first;
				if (_oppStrikeTime == std::nullopt || item.second.value() < _oppStrikeTime.value())
				{
					bestCollisionTimeId = item.first;
				};
			}				
		}

		if (bestCollisionTimeId >= 0 || isCollisionTime && !defender.touch)
		{
			const auto afterCollisionTick = UpdateBallEntities(collisionTimes[bestCollisionTimeId].value(), bestBallVelocities[bestCollisionTimeId]);
			for (auto robot:myRobots)
			{
				if (!robot.touch) continue;

				Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
				std::optional<double> collisionT = std::nullopt;
				const Action robotAction = SetAttackerAction(
					robot, game.ball, isMeGoalPossible, afterCollisionTick + AttackerAddTicks, 
					robot.id == defender.id ? _myGates : _beforeMyGates,
					collisionT, bestBallVelocity);
				collisionTimes[robot.id] = collisionT;
				bestBallVelocities[robot.id] = bestBallVelocity;
				_actions[robot.id] = robotAction;
			}
		}
		else if (isCollisionTime)
		{
			for (auto robot:myRobots)
			{
				if (!robot.touch) continue;
				Action robotAction = GetDefaultAction(robot, robot.id == defender.id ? _myGates : _beforeMyGates);
				collisionTimes[robot.id] = std::nullopt;
				bestBallVelocities[robot.id] = Helper::GetBallVelocity(game.ball);
				_actions[robot.id] = robotAction;
			}			
		}
		else
		{
			if (defender.touch)
			{
				Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
				std::optional<double> collisionT = std::nullopt;
				Action robotAction = SetDefenderAction(defender, game.ball, _myGates, isMeGoalPossible, collisionT, bestBallVelocity);
				collisionTimes[defender.id] = collisionT;
				bestBallVelocities[defender.id] = bestBallVelocity;
				_actions[defender.id] = robotAction;
			}
			for (auto attacker:attackes)
			{
				if (!attacker.touch) continue;

				Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
				std::optional<double> collisionT = std::nullopt;
				Action robotAction = SetAttackerAction(attacker, game.ball, isMeGoalPossible, 1, _beforeMyGates, collisionT, bestBallVelocity);
				collisionTimes[attacker.id] = collisionT;
				bestBallVelocities[attacker.id] = bestBallVelocity;
				_actions[attacker.id] = robotAction;
			}
		}
	}
	else
	{
		Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
		std::optional<double> collisionT = std::nullopt;
		Action robotAction = SetDefenderAction(defender, game.ball, _myGates, isMeGoalPossible, collisionT, bestBallVelocity);
		collisionTimes[defender.id] = collisionT;
		bestBallVelocities[defender.id] = bestBallVelocity;
		_actions[defender.id] = robotAction;

		if (collisionT != std::nullopt && (_oppStrikeTime == std::nullopt || collisionT.value() < _oppStrikeTime.value()))
		{
			const auto afterCollisionTick = UpdateBallEntities(collisionTimes[defender.id].value(), bestBallVelocities[defender.id]);

			for (auto attacker : attackes)
			{
				Vector3D attBestBallVelocity = Helper::GetBallVelocity(game.ball);
				std::optional<double> attCollisionT = std::nullopt;
				const Action attRobotAction = SetAttackerAction(
					attacker, game.ball, isMeGoalPossible, afterCollisionTick + AttackerAddTicks, _beforeMyGates, attCollisionT, attBestBallVelocity);
				collisionTimes[attacker.id] = attCollisionT;
				bestBallVelocities[attacker.id] = attBestBallVelocity;
				_actions[attacker.id] = attRobotAction;
			}
		}
		else
		{
			int bestAttCollisionTId = -1;

			for (auto attacker : attackes)
			{
				Vector3D attBestBallVelocity = Helper::GetBallVelocity(game.ball);
				std::optional<double> attCollisionT = std::nullopt;
				const Action attRobotAction = SetAttackerAction(
					attacker, game.ball, isMeGoalPossible, 1, _beforeMyGates, attCollisionT, attBestBallVelocity);

				if (attCollisionT != std::nullopt && (bestAttCollisionTId == -1 || attCollisionT.value() < collisionTimes[bestAttCollisionTId].value()))
				{
					bestAttCollisionTId = attacker.id;
				}

				collisionTimes[attacker.id] = attCollisionT;
				bestBallVelocities[attacker.id] = attBestBallVelocity;
				_actions[attacker.id] = attRobotAction;
			}

			if (bestAttCollisionTId >= 0 &&
				(_oppStrikeTime == std::nullopt || collisionTimes[bestAttCollisionTId].value() < _oppStrikeTime.value()))
			{
				const auto afterCollisionTick = UpdateBallEntities(collisionTimes[bestAttCollisionTId].value(), bestBallVelocities[bestAttCollisionTId]);
				
				Vector3D defBestBallVelocity = Helper::GetBallVelocity(game.ball);
				std::optional<double> defCollisionT = std::nullopt;
				//TODO: защитник должен возвращаться на _gates, а не _beforeGates
				const Action defRobotAction = SetAttackerAction(
					defender, game.ball, isMeGoalPossible, afterCollisionTick + AttackerAddTicks, _myGates, defCollisionT, defBestBallVelocity);
				collisionTimes[defender.id] = defCollisionT;
				bestBallVelocities[defender.id] = defBestBallVelocity;
				_actions[defender.id] = defRobotAction;				
			}
			else
			{
				//TODO: если защитник может как-то ударить, идти напом в атаку

				//выбиваем оптимальный Action. Action другого робота сбрасываем на дефолтный
				for (Robot robot : myRobots)
				{
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
						if (_oppStrikeTime == std::nullopt ||
							std::max(*collisionTimes[robot.id], *collisionTimes[otherRobot.id]) < _oppStrikeTime.value())
						{
							if (!otherRobot.touch ||  //если один уже в воздухе, оптимизация не имеет смысла
								CompareBallVelocities(bestBallVelocities[robot.id], bestBallVelocities[otherRobot.id]) > 0)
							{
								_actions[robot.id] =
									GetDefaultAction(robot, robot.z < otherRobot.z ? _myGates : _beforeMyGates);
							}
						}

						else if (*collisionTimes[robot.id] > *collisionTimes[otherRobot.id])
						{
							_actions[robot.id] =
								GetDefaultAction(robot, robot.z < otherRobot.z ? _myGates : _beforeMyGates);
						}
					}
				}
			}

		}
	}



	//std::optional<double> collisionT = std::nullopt;
	//if (!defender.touch)
	//{
	//	InitJumpingRobotAction(defender, game.ball, collisionTimes, bestBallVelocities);
	//}
	//else
	//{
	//	Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
	//	Action robotAction = SetDefenderAction(defender, game.ball, _myGates, isMeGoalPossible, collisionT, bestBallVelocity);
	//	collisionTimes[defender.id] = collisionT;
	//	bestBallVelocities[defender.id] = bestBallVelocity;
	//	_actions[defender.id] = robotAction;
	//}


	//if (collisionTimes[defender.id] == std::nullopt || _oppStrikeTime != std::nullopt && collisionTimes[defender.id].value() > _oppStrikeTime.value())
	//{
	//	//действуем по-старинке
	//	
	//	for (auto attacker : attackes)
	//	{
	//		collisionT = std::nullopt;
	//		if (!attacker.touch)
	//		{
	//			InitJumpingRobotAction(attacker, game.ball, collisionTimes, bestBallVelocities);
	//		}
	//		else
	//		{

	//			Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
	//			Action robotAction = SetAttackerAction(attacker, game.ball, isMeGoalPossible, 1, collisionT, bestBallVelocity);
	//			collisionTimes[attacker.id] = collisionT;
	//			bestBallVelocities[attacker.id] = bestBallVelocity;
	//			_actions[attacker.id] = robotAction;
	//		}
	//	}

	//	//выбиваем оптимальный Action. Action другого робота сбрасываем на дефолтный
	//	for (Robot robot : game.robots)
	//	{
	//		if (!robot.is_teammate) continue;
	//		if (!robot.touch) continue;

	//		Robot otherRobot{};
	//		for (Robot r : game.robots)
	//		{
	//			if (!r.is_teammate || r.id == robot.id) continue;
	//			otherRobot = r;
	//			break;
	//		}

	//		if (collisionTimes[robot.id] != std::nullopt && collisionTimes[otherRobot.id] != std::nullopt)
	//		{
	//			if (_oppStrikeTime == std::nullopt ||
	//				std::max(*collisionTimes[robot.id], *collisionTimes[otherRobot.id]) < _oppStrikeTime.value())
	//			{
	//				if (!otherRobot.touch ||  //если один уже в воздухе, оптимизация не имеет смысла
	//					CompareBallVelocities(bestBallVelocities[robot.id], bestBallVelocities[otherRobot.id]) > 0)
	//				{
	//					_actions[robot.id] =
	//						GetDefaultAction(robot, robot.z < otherRobot.z ? _myGates : _beforeMyGates);
	//				}
	//			}

	//			else if (*collisionTimes[robot.id] > *collisionTimes[otherRobot.id])
	//			{
	//				_actions[robot.id] =
	//					GetDefaultAction(robot, robot.z < otherRobot.z ? _myGates : _beforeMyGates);
	//			}
	//		}
	//	}
	//}
	//else
	//{
	//	const auto afterCollisionTick = UpdateBallEntities(collisionTimes[defender.id].value(), bestBallVelocities[defender.id]);

	//	for (auto attacker : attackes)
	//	{
	//		collisionT = std::nullopt;
	//		if (!attacker.touch)
	//		{
	//			InitJumpingRobotAction(attacker, game.ball, collisionTimes, bestBallVelocities);
	//		}
	//		else
	//		{
	//			Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
	//			Action robotAction = SetAttackerAction(
	//				attacker, game.ball, isMeGoalPossible, afterCollisionTick + AttackerAddTicks, collisionT, bestBallVelocity);
	//			collisionTimes[attacker.id] = collisionT;
	//			bestBallVelocities[attacker.id] = bestBallVelocity;
	//			_actions[attacker.id] = robotAction;
	//		}
	//	}
	//}

		
	
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

	_hitEs[0]= Constants::Rules.MIN_HIT_E; 
	_hitEs[1]= Constants::Rules.MAX_HIT_E; 
}

void MyStrategy::InitAction(model::Action& action, int id)
{
	action.jump_speed = _actions[id].jump_speed;
	action.target_velocity_x = _actions[id].target_velocity_x;
	action.target_velocity_y = _actions[id].target_velocity_y;
	action.target_velocity_z = _actions[id].target_velocity_z;
	action.use_nitro = _actions[id].use_nitro;
}

void MyStrategy::InitJumpingRobotAction(const Robot& robot,  const Ball& ball, 
	std::map<int, std::optional<double>>& collisionTimes, std::map<int, Vector3D>& bestBallVelocities)
{
	Action robotAction = model::Action();

	RobotEntity re = RobotEntity(robot);
	model::Action reAction = Action();
	reAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
	re.Action = reAction;

	Simulator::Tick(re, BallEntity(ball));
	if (!re.IsArenaCollided) robotAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
	_actions[robot.id] = robotAction;
	auto collisionT = Simulator::GetCollisionT(
		Helper::GetRobotPosition(robot),
		Helper::GetRobotVelocity(robot),
		Helper::GetBallPosition(ball),
		Helper::GetBallVelocity(ball));
	collisionTimes[robot.id] = collisionT;

	if (collisionT != std::nullopt)
	{
		auto collisionBe = BallEntity(ball);
		auto collisionRe = RobotEntity(robot);
		collisionRe.Action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
		const auto isCollision = SimulateNoTouchEntitiesCollision(collisionBe, collisionRe, collisionT.value());
		if (isCollision)
		{
			auto delta = collisionBe.Position - collisionRe.Position;
			auto dl = delta.Length();

			const auto hitE = (_hitEs[0] + _hitEs[1]) / 2;
			bool isGoalScored;
			Simulator::Update(collisionRe, collisionBe,
				1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored);
			bestBallVelocities[robot.id] = collisionBe.Velocity;
		}//TODO: если !isCollision, надо сбросить bestBallVelocities[defender.id] в null
	}
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

BallEntity MyStrategy::SimulateTickBall(const BallEntity & ballEntity, bool & isGoalScored) const
{
	BallEntity bec = BallEntity(ballEntity);
	Simulator::Update(bec, 1.0 / Constants::Rules.TICKS_PER_SECOND, isGoalScored);
	if (!bec.IsArenaCollided)
	{
		return bec;
	}

	bec = BallEntity(ballEntity);

	Simulator::Tick(bec);
	bec.IsArenaCollided = false;
	return bec;
}

void MyStrategy::SimulateTickRobot(RobotEntity & robotEntity, bool& isArenaCollided) const
{
	RobotEntity rec = RobotEntity(robotEntity);
	bool isGoalScored;
	Simulator::Update(rec, 1.0 / Constants::Rules.TICKS_PER_SECOND, isGoalScored);
	if (!rec.IsArenaCollided)
	{
		robotEntity = rec;
		return;
	}
	
	isArenaCollided = true;
	Simulator::Tick(robotEntity);
	robotEntity.IsArenaCollided = false; 
	//сохраним флаг, коллизии робота с ареной, чтобы использовать потом (например, отбрасывать такие точки для защитника)
}

bool MyStrategy::SimulateCollision(BallEntity & ballEntity, RobotEntity & robotEntity,
	std::optional<double>& collisionT) const
{
	auto const targetVelocity = Helper::GetTargetVelocity(robotEntity.Position.X, 0, robotEntity.Position.Z,
		ballEntity.Position.X, 0, ballEntity.Position.Z,
		Constants::Rules.ROBOT_MAX_GROUND_SPEED);

	model::Action action = model::Action();
	action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
	action.target_velocity_x = targetVelocity.X;
	action.target_velocity_y = 0.0;
	action.target_velocity_z = targetVelocity.Z;
	action.use_nitro = false;

	robotEntity.Action = action;	

	//симулируем отрыв от земли - на это надо 2 микротика
	bool isGoalScored = false;
	Simulator::Update(robotEntity, ballEntity,
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
	Simulator::Update(robotEntity, ballEntity,
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
	//const auto mtTime = 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;
	//for (int i = 0; i < 2; ++i)
	//{
	//	Simulator::Update(robotEntity,
	//		mtTime,
	//		isGoalScored);
	//}
	////TODO: здесь теоретически мяч может врезаться в стену	
	//ballEntity.Position.X += ballEntity.Velocity.X * 2 * mtTime;
	//ballEntity.Position.Y += ballEntity.Velocity.Y * 2 * mtTime - Constants::Rules.GRAVITY * (2 * mtTime) * (2 * mtTime) / 2;
	//ballEntity.Position.Z += ballEntity.Velocity.Z * 2 * mtTime;
	//ballEntity.Velocity.Y -= Constants::Rules.GRAVITY * 2 * mtTime;


	//считаем время коллизии
	collisionT = Simulator::GetCollisionT(robotEntity.Position, robotEntity.Velocity, ballEntity.Position,
		ballEntity.Velocity);
	if (collisionT == std::nullopt) return false;

	return SimulateNoTouchEntitiesCollision(ballEntity, robotEntity, collisionT.value());
	
}

bool MyStrategy::SimulateNoTouchEntitiesCollision(BallEntity & ballEntity, RobotEntity & robotEntity, double collisionT) const
{
	robotEntity.IsArenaCollided = false;

	//ситуаци¤ за микротик до коллизии.	
	auto const collisionMicroTicks = int(collisionT * Constants::Rules.TICKS_PER_SECOND * Constants::Rules.MICROTICKS_PER_TICK);
	auto const t = collisionMicroTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;

	bool needReSimulation = false;
	auto const rePosition = Vector3D(
		robotEntity.Position.X + robotEntity.Velocity.X * t,
		robotEntity.Position.Y + robotEntity.Velocity.Y * t - Constants::Rules.GRAVITY * t * t / 2,
		robotEntity.Position.Z + robotEntity.Velocity.Z * t);
	Dan dan = DanCalculator::GetDanToArena(rePosition, Constants::Rules.arena);
	if (robotEntity.Radius - dan.Distance > 0) needReSimulation = true;
	if (needReSimulation) return false;//если робот ударится о стену, не рассматриваем такой вариант

	robotEntity.Position = rePosition;
	robotEntity.Velocity.Y -= Constants::Rules.GRAVITY * t;


	bool needBeSimulation = false;
	const auto bePosition = Vector3D(
		ballEntity.Position.X + ballEntity.Velocity.X * t,
		ballEntity.Position.Y + ballEntity.Velocity.Y * t - Constants::Rules.GRAVITY * t * t / 2,
		ballEntity.Position.Z + ballEntity.Velocity.Z * t);
	dan = DanCalculator::GetDanToArena(bePosition, Constants::Rules.arena);
	if (ballEntity.Radius - dan.Distance > 0) needBeSimulation = true;

	if (!needBeSimulation)
	{
		ballEntity.Position = bePosition;
		ballEntity.Velocity.Y -= Constants::Rules.GRAVITY * t;
		return true;
	}
	bool isGoalScored;
	auto const collisionTicks = int(collisionT * Constants::Rules.TICKS_PER_SECOND);
	for (int i = 1; i <= collisionTicks; ++i)
	{
		ballEntity = SimulateTickBall(ballEntity, isGoalScored);
	}

	auto const timeLeft = collisionT - collisionTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND;
	auto const mtLeft = int(timeLeft * Constants::Rules.TICKS_PER_SECOND * Constants::Rules.MICROTICKS_PER_TICK);
	auto const mtLeftTime = mtLeft * 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;

	//TODO: Неточно, т.к. может быть коллизи¤ м¤ча с ареной или коллизи¤ робота с ареной
	Simulator::Update(ballEntity,
		mtLeftTime,
		isGoalScored);

	return true;
}

bool MyStrategy::IsPenaltyArea(const Vector3D & position, bool isDefender) const
{
	/*if (isDefender)
		return position.Z <= _penaltyAreaZ;*/
	return  position.Z < 0;
}

double MyStrategy::GetVectorAngleToHorizontal(const Vector3D & v) const
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
	if (v1HorAngle > M_PI / 9 && v2HorAngle > M_PI / 9)
		return abs(v1.Z) > abs(v2.value().Z) ? -1 : 1;
	return v1.Y > v2.value().Y ? -1 : 1;
}

bool MyStrategy::IsGoalBallDirection2(const BallEntity & startBallEntity, int directionCoeff) const
{
	if (startBallEntity.Velocity.Z * directionCoeff <= 0) return false;
	if (abs(startBallEntity.Velocity.Z) < EPS) return false;

	const auto distToGates = abs(Constants::Rules.arena.depth / 2 * directionCoeff - startBallEntity.Position.Z);
	const auto timeToGates = distToGates / abs(startBallEntity.Velocity.Z);

	const auto distToSide = startBallEntity.Velocity.X > 0 ?
		abs(Constants::Rules.arena.width / 2 - startBallEntity.Position.X) :
		abs(-Constants::Rules.arena.width / 2 - startBallEntity.Position.X);
	if (abs(startBallEntity.Velocity.X) > EPS)
	{
		const auto timeToSide = distToSide / abs(startBallEntity.Velocity.X);
		if (timeToSide < timeToGates) return false;
	}

	const auto xForTimeToGates = startBallEntity.Position.X + startBallEntity.Velocity.X * timeToGates;
	if (abs(xForTimeToGates) > Constants::Rules.arena.goal_width / 2 + Constants::Rules.arena.goal_side_radius + Constants::Rules.BALL_RADIUS)
		return false;

	const auto a = 1;
	const auto b1 = -startBallEntity.Velocity.Y / Constants::Rules.GRAVITY;
	const auto c = 2 * (Constants::Rules.arena.height - startBallEntity.Position.Y) / Constants::Rules.GRAVITY;
	const auto d1 = b1 * b1 - a * c;
	if (d1 > 0)
	{
		const auto timeToRoof = -b1 - sqrt(d1);
		if (timeToRoof >= 0 && timeToRoof < timeToGates) 
			return false;
	}

	BallEntity ballEntity = BallEntity(startBallEntity);

	bool isGoalScored = false;
	double z = ballEntity.Position.Z;

	for (int t = 1; t <= BallMoveTicks; ++t)
	{
		ballEntity = SimulateTickBall(ballEntity, isGoalScored);
		if (isGoalScored) return true;
		const auto isZIncreasing = ballEntity.Position.Z * directionCoeff > z * directionCoeff;		
		if (!isZIncreasing) return false;
		z = ballEntity.Position.Z;
	}	
	return false;
}

model::Action MyStrategy::SetDefenderAction(
	const model::Robot & me, const model::Ball & ball, const Vector3D & defenderPoint,
	bool isMeGoalPossible,
	std::optional<double>& collisionT, Vector3D& bestBallVelocity)
{
	model::Action action = model::Action();
	Vector3D targetVelocity;
	
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
		true,
		jumpCollisionT,
		jumpBallVelocity))
	{
		if (defendPoint == std::nullopt || CompareBallVelocities(jumpBallVelocity.value(), bestBallVelocity) < 0)
		{
			collisionT = jumpCollisionT;
			bestBallVelocity = jumpBallVelocity.value();

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
		targetVelocity = GetDefendPointTargetVelocity(me, defenderPoint);
	}
	else
	{
		targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, defendPoint.value().X, 0, defendPoint.value().Z,
			Constants::Rules.ROBOT_MAX_GROUND_SPEED);
	}

	action.target_velocity_x = targetVelocity.X;
	action.target_velocity_y = 0.0;
	action.target_velocity_z = targetVelocity.Z;
	action.jump_speed = 0.0;
	action.use_nitro = false;
	return action;
}

std::optional<Vector3D> MyStrategy::GetDefenderStrikeBallVelocity(const model::Robot & robot, int t,
	int startAttackTick,
	bool isMeGoalPossible,
	bool isDefender,
	std::optional<double>& collisionT, bool & isPassedBy)
{
	std::optional<Vector3D> bestBallVelocity = std::nullopt;
	isPassedBy = false;
	const BallEntity ballEntity = _ballEntities[t];

	if (!IsPenaltyArea(ballEntity.Position, isDefender))
	{
		return std::nullopt;
	}

	Vector3D targetPos = Vector3D(ballEntity.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
		ballEntity.Position.Z);

	for (int moveT = startAttackTick; moveT < t; ++moveT)
	{
		if (bestBallVelocity != std::nullopt &&
			_oppStrikeTime.has_value() && 
			moveT * 1.0 / Constants::Rules.TICKS_PER_SECOND > _oppStrikeTime.value())
			return bestBallVelocity;

		PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
			Helper::GetRobotPosition(robot),
			targetPos,
			Helper::GetRobotVelocity(robot),
			moveT,
			1.0);

		if (pvContainer.IsPassedBy)
		{
			isPassedBy = true;
			return bestBallVelocity; // проскочим целевую точку. дальше все непредсказуемо
		}

		if (_ballEntities[moveT].Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)//м¤ч в воротах
			return bestBallVelocity;

		std::optional<double> curJumpCollisionT = std::nullopt;
		std::optional<Vector3D> collisionBallVelocity = std::nullopt;

		if (IsOkDefenderPosToJump(pvContainer.Position, pvContainer.Velocity,
			_ballEntities[moveT].Position, _ballEntities[moveT].Velocity,
			isMeGoalPossible,
			isDefender,
			curJumpCollisionT, collisionBallVelocity))
		{			

			if (CompareBallVelocities(*collisionBallVelocity, bestBallVelocity) < 0)
			{
				double curCollisionT = moveT * 1.0 / Constants::Rules.TICKS_PER_SECOND + curJumpCollisionT.value();
				if (bestBallVelocity != std::nullopt &&
					_oppStrikeTime.has_value() &&
					curCollisionT > _oppStrikeTime.value())
					continue;

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
	bool isDefender,
	std::optional<double>& jumpCollisionT, std::optional<Vector3D>& collisionBallVelocity)
{
	auto moveTBallEntity = BallEntity(Vector3D(moveTBePosition), Vector3D(moveTBeVelocity));
	if (moveTBallEntity.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
	{
		return false;
	}

	auto robotEntity = RobotEntity(
		Vector3D(robotPosition), Vector3D(robotVelocity), Constants::Rules.ROBOT_MIN_RADIUS,
		true, Vector3D(0, 1, 0), 0.0);

	std::optional<double> jumpCollisionTCur = std::nullopt;
	auto const isCollision = SimulateCollision(moveTBallEntity, robotEntity, jumpCollisionTCur);
	if (!isCollision) return false;
	
	if (moveTBallEntity.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
		return false;

	if (robotEntity.Velocity.Y < 0) return false;//не бьем на излете
	if (robotEntity.Position.Y >= moveTBallEntity.Position.Y) return false;//не бьем сверху-вниз
	if (robotEntity.IsArenaCollided) return false;

	//коллизи¤ вне штрафной площади
	if (!IsPenaltyArea(moveTBallEntity.Position, isDefender))
		return false;
	
	bool isGoalScored;
	for (double hitE : _hitEs)
	{
		RobotEntity re2 = RobotEntity(robotEntity);
		BallEntity ballEntity2 = BallEntity(moveTBallEntity);

		Simulator::Update(re2, ballEntity2,
			1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored);

		if (ballEntity2.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
			return false;

		if (!ballEntity2.IsCollided) //веро¤тно, м¤ч ударилс¤ об арену. прыгать смысла нет
		{
			return false;
		}		

		if (ballEntity2.Velocity.Z < 0)
		{
			if (!isMeGoalPossible) return false;
			const auto isCollisionGoalPossible = IsGoalBallDirection2(ballEntity2, -1);
			if (isCollisionGoalPossible) return false;
		}

		/*if (collisionBallVelocity == std::nullopt || ballEntity2.Velocity.Z < collisionBallVelocity.value().Z)
			collisionBallVelocity = ballEntity2.Velocity;*/
	}

	auto const hitE = (_hitEs[0] + _hitEs[1]) / 2.0;
	RobotEntity re3 = RobotEntity(robotEntity);
	BallEntity ballEntity3 = BallEntity(moveTBallEntity);
	Simulator::Update(re3, ballEntity3,
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored);
	collisionBallVelocity = ballEntity3.Velocity;

	jumpCollisionT = jumpCollisionTCur;
	return true;
}

std::optional<Vector3D> MyStrategy::GetDefenderMovePoint(const model::Robot & robot, const model::Ball & ball,
	bool isMeGoalPossible,
	std::optional<double>& collisionT, Vector3D& bestBallVelocity)
{
	std::optional<Vector3D> movePoint = std::nullopt;

	int bestT = -1;

	for (int t = 1; t <= BallMoveTicks; ++t)
	{
		auto ballEntity = _ballEntities[t];
		
		std::optional<double> curCollisionT = std::nullopt;
		bool isPassedBy = false;
		std::optional<Vector3D> ballVelocity = 
			GetDefenderStrikeBallVelocity(robot, t, 1, isMeGoalPossible, true, curCollisionT, isPassedBy);
		//if (isPassedBy)//TODO: возможно, следующие точки будут лучше
		//	continue;
		//	return movePoint;

		if (ballVelocity == std::nullopt) continue;
		if (CompareBallVelocities(*ballVelocity, bestBallVelocity) < 0)
		{			
			bestBallVelocity = ballVelocity.value();
			collisionT = curCollisionT.value();

			movePoint = Vector3D(_ballEntities[t].Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
				_ballEntities[t].Position.Z);
			bestT = t;
		}
	}

	if(movePoint != std::nullopt)
	{
		_drawSpheres.push_back(Sphere((*movePoint).X, (*movePoint).Y, (*movePoint).Z, 0.25, 0, 0, 1, 0.5));
		_drawSpheres.push_back(Sphere(
			_ballEntities[bestT].Position.X, _ballEntities[bestT].Position.Y, _ballEntities[bestT].Position.Z,
			Constants::Rules.BALL_RADIUS, 0, 0, 1, 0.5));
	}
	
	return movePoint;
}

model::Action MyStrategy::SetAttackerAction(const model::Robot & me, const model::Ball & ball,
	bool isMeGoalPossible,
	int startAttackTick,
	const Vector3D& defenderPoint,
	std::optional<double>& collisionT, Vector3D& bestBallVelocity)
{
	model::Action action = model::Action();

	Vector3D targetVelocity;
	RobotEntity robotEntity = RobotEntity(me);

	std::optional<double> jumpCollisionT = std::nullopt;
	std::optional<Vector3D> jumpBallVelocity = std::nullopt;

	if (startAttackTick == 1 && IsOkPosToJump(BallEntity(ball), robotEntity, jumpCollisionT, jumpBallVelocity))
	{
		collisionT = jumpCollisionT;
		bestBallVelocity = jumpBallVelocity.value();

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
	std::optional<Vector3D> movePoint = 
		GetAttackerMovePoint(me, ball,isMeGoalPossible, startAttackTick, collisionT, isDefender, bestBallVelocity);
	if (isDefender)
	{
		jumpCollisionT = std::nullopt;
		jumpBallVelocity = std::nullopt;

		auto const ballEntity = BallEntity(ball);
		auto const isGoalPossible = IsGoalBallDirection2(ballEntity, -1);
		if (startAttackTick == 1 && IsOkDefenderPosToJump(
			Helper::GetRobotPosition(me),
			Helper::GetRobotVelocity(me),
			Helper::GetBallPosition(ball),
			Helper::GetBallVelocity(ball),
			isGoalPossible,
			false,
			jumpCollisionT, jumpBallVelocity))
		{
			if (movePoint == std::nullopt || CompareBallVelocities(jumpBallVelocity.value(), bestBallVelocity) < 0)
			{
				collisionT = jumpCollisionT;
				bestBallVelocity = jumpBallVelocity.value();

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
			targetVelocity = GetDefendPointTargetVelocity(me, defenderPoint);
		}
		else
		{
			targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, movePoint.value().X, 0, movePoint.value().Z,
				Constants::Rules.ROBOT_MAX_GROUND_SPEED);
		}

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
			if (&defenderPoint == &_beforeMyGates && //только нап бежит за мячом				
				Helper::GetLength2(Helper::GetBallPosition(ball), Helper::GetRobotPosition(me)) >
				_distToFollowBall * _distToFollowBall
				&&	me.z < ball.z)
			{
				movePoint = Vector3D(ball.x, 0, ball.z);
			}
			else
			{
				movePoint = defenderPoint;
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
	int t, int directionCoeff, std::optional<double>& collisionT, std::optional<Vector3D>& bestBallVelocity)
{
	collisionT = std::nullopt;
	bestBallVelocity = std::nullopt;

	PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
		Helper::GetRobotPosition(robot),
		mePos,
		Helper::GetRobotVelocity(robot),
		t,
		1.0);

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
		return false; //столкнемс¤ еще при движении в точку

	RobotEntity robotE = RobotEntity(
		Vector3D(pvContainer.Position),
		Vector3D(pvContainer.Velocity),
		Constants::Rules.ROBOT_MIN_RADIUS,
		true,
		Vector3D(0, 1, 0),
		0);

	std::optional<Vector3D> ballVelocity = std::nullopt;
	std::optional<double> jumpCollisionT = std::nullopt;

	const bool isOkPosToStrike = directionCoeff == 1 ?
		IsOkPosToJump(BallEntity(ballEntity), robotE, jumpCollisionT, ballVelocity) :
		IsOkOppPosToJump(BallEntity(ballEntity), robotE, jumpCollisionT);

	if (isOkPosToStrike)
	{
		if (directionCoeff == 1)
		{
			const auto color = 0;
			_drawSpheres.push_back(Sphere(mePos.X, mePos.Y, mePos.Z, 0.25, color, color, color, 0.5));
			_drawSpheres.push_back(Sphere(pvContainer.Position.X, pvContainer.Position.Y, pvContainer.Position.Z, 1,
				color, color, color, 0.5));
			_drawSpheres.push_back(Sphere(ballEntity.Position.X, ballEntity.Position.Y, ballEntity.Position.Z,
				Constants::Rules.BALL_RADIUS, 0, 1, 0, 0.5));
		}
		collisionT = jumpCollisionT;
		bestBallVelocity = ballVelocity;
		return true;
	}

	return false;
}

std::optional<Vector3D> MyStrategy::GetAttackerMovePoint(const model::Robot & robot, const model::Ball & ball,
	bool isMeGoalPossible,
	int startAttackTick,
	std::optional<double>& collisionT, bool & isDefender, Vector3D& bestBallVelocity)
{
	std::optional<Vector3D> movePoint = std::nullopt;
	isDefender = false;

	int bestT = -1;
	for (int t = startAttackTick; t <= startAttackTick + BallMoveTicks; ++t)
	{
		if (_goalScoringTick >= 0 && t >= _goalScoringTick)
			return movePoint;

		auto ballEntity = _ballEntities[t];		

		if (IsPenaltyArea(ballEntity.Position, false))//включаем режим защитника
		{
			isDefender = true;
			std::optional<double> curCollisionT = std::nullopt;
			bool isPassedBy = false;
			std::optional<Vector3D> ballVelocity = 
				GetDefenderStrikeBallVelocity(robot, t, startAttackTick, isMeGoalPossible, false, curCollisionT, isPassedBy);
			/*if (isPassedBy)
				continue;*/
				//return movePoint;

			if (ballVelocity == std::nullopt) continue;
			if (CompareBallVelocities(ballVelocity.value(), bestBallVelocity) < 0)
			{
				bestBallVelocity = ballVelocity.value();
				collisionT = curCollisionT.value();

				movePoint = Vector3D(_ballEntities[t].Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
					_ballEntities[t].Position.Z);
				bestT = t;
			}
		}
		else
		{
			std::optional<double> jumpCollisionT = std::nullopt;
			std::optional<Vector3D> ballVelocity = std::nullopt;
			const auto attackMovePoint = GetAttackerStrikePoint(robot, t, 1, jumpCollisionT, ballVelocity);
			if (attackMovePoint != std::nullopt)
			{
				if (jumpCollisionT == std::nullopt) throw "jumpCollisionT is null";
				bestBallVelocity = ballVelocity.value();
				collisionT = t * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();
				isDefender = false;
				return attackMovePoint;
			}
		}
	}


	if (movePoint != std::nullopt)
	{
		_drawSpheres.push_back(Sphere((*movePoint).X, (*movePoint).Y, (*movePoint).Z, 0.25, 1, 0, 0, 0.5));
		_drawSpheres.push_back(Sphere(
			_ballEntities[bestT].Position.X, _ballEntities[bestT].Position.Y, _ballEntities[bestT].Position.Z,
			Constants::Rules.BALL_RADIUS, 1, 0, 0, 0.5));
	}

	return movePoint;
}

std::optional<Vector3D> MyStrategy::GetAttackerStrikePoint(const model::Robot & robot, int t,
	int directionCoeff,
	std::optional<double>& collisionT, std::optional<Vector3D>& bestBallVelocity)
{
	const BallEntity ballEntity = _ballEntities[t];
	//if (_beforeStrikePoints.count(robot.id) > 0) //TODO: учесть время t - не всегда надо проверять
	//{
	//	if (IsOkPosToMove(_beforeStrikePoints[robot.id], robot, ballEntity, t, directionCoeff, collisionT))
	//		return _beforeStrikePoints[robot.id];
	//	_beforeStrikePoints.erase(robot.id);
	//}

	//код, чтобы сразу отбросить лишние варианты. TODO: оптимизировать
	if (directionCoeff == 1)
	{
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
	if (IsOkPosToMove(mePos, robot, ballEntity, t, directionCoeff, collisionT, bestBallVelocity))
	{
		_beforeStrikePoints[robot.id] = mePos;
		return mePos;
	}

	for (int i = 1; i <= StrikeSphereStepsCount; ++i)
	{
		Vector3D rVectorMult1 = rVector1 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos1 = Vector3D(ballEntity.Position.X + rVectorMult1.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult1.Z);
		if (IsOkPosToMove(mePos1, robot, ballEntity, t, directionCoeff, collisionT, bestBallVelocity))
		{
			_beforeStrikePoints[robot.id] = mePos1;
			return mePos1;
		}

		Vector3D rVectorMult2 = rVector2 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos2 = Vector3D(ballEntity.Position.X + rVectorMult2.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult2.Z);
		if (IsOkPosToMove(mePos2, robot, ballEntity, t, directionCoeff, collisionT, bestBallVelocity))
		{
			_beforeStrikePoints[robot.id] = mePos2;
			return mePos2;
		}
	}

	return std::nullopt;
}

bool MyStrategy::IsOkPosToJump(
	BallEntity ballEntity, RobotEntity & robotEntity, 
	std::optional<double>& collisionT, std::optional<Vector3D>& bestBallVelocity)
{
	const auto directionCoeff = 1;

	if (ballEntity.Velocity.Z <=0 && Helper::GetLength2(robotEntity.Position, ballEntity.Position) > _maxStrikeDist2 + EPS)
	{
		collisionT = std::nullopt;
		return false;
	}

	//auto const prevBallEntity = BallEntity(ballEntity);

	std::optional<double> jumpCollisionT = std::nullopt;
	auto const isCollision = SimulateCollision(ballEntity, robotEntity, jumpCollisionT);
	if (!isCollision) return false;

	//симулируем коллизию
	
	bool isGoalScored;
	for(double hitE: _hitEs)
	{
		RobotEntity re2 = RobotEntity(robotEntity);
		BallEntity ballEntity2 = BallEntity(ballEntity);
		Simulator::Update(re2, ballEntity2, 
			1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored);

		if (!ballEntity2.IsCollided) //веро¤тно, м¤ч ударилс¤ об арену. прыгать смысла нет
		{
			return false;
		}

		double angle = GetVectorAngleToHorizontal(ballEntity2.Velocity);
		if (angle * directionCoeff > M_PI / 3) return false; //не бьем под большим углом к горизонтали

		if (!IsGoalBallDirection2(ballEntity2, directionCoeff)) return false;
		/*double velocityZ = ballEntity2.Velocity.Z;
		if (velocityZ * directionCoeff < prevBallEntity.Velocity.Z * directionCoeff)
			if (IsGoalBallDirection2(prevBallEntity, directionCoeff)) 
				return false;*/


	}
	collisionT = jumpCollisionT;

	auto const hitE = (_hitEs[0] + _hitEs[1]) / 2.0;
	RobotEntity re3 = RobotEntity(robotEntity);
	BallEntity ballEntity3 = BallEntity(ballEntity);
	Simulator::Update(re3, ballEntity3,
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored);
	bestBallVelocity = ballEntity3.Velocity;

	return true;
}

bool MyStrategy::IsOkOppPosToJump(BallEntity ballEntity, RobotEntity & robotEntity, std::optional<double>& collisionT) const
{
	const auto directionCoeff = -1;
	//auto const prevBallEntity = BallEntity(ballEntity);

	std::optional<double> jumpCollisionT = std::nullopt;
	auto const isCollision = SimulateCollision(ballEntity, robotEntity, jumpCollisionT);
	if (!isCollision) return false;

	//симулируем коллизию

	bool isGoalScored;
	for (double hitE : _hitEs)
	{
		RobotEntity re2 = RobotEntity(robotEntity);
		BallEntity ballEntity2 = BallEntity(ballEntity);
		Simulator::Update(re2, ballEntity2,
			1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored);

		if (!ballEntity2.IsCollided) //веро¤тно, м¤ч ударилс¤ об арену. прыгать смысла нет
		{
			return false;
		}

		if (!IsGoalBallDirection2(ballEntity2, directionCoeff)) return false;
		/*double velocityZ = ballEntity2.Velocity.Z;
		if (velocityZ * directionCoeff < prevBallEntity.Velocity.Z * directionCoeff)
			if (IsGoalBallDirection2(prevBallEntity, directionCoeff))
				return false;*/

	}

	collisionT = jumpCollisionT;
	return true;
}

Vector3D MyStrategy::GetDefendPointTargetVelocity(const model::Robot & robot, const Vector3D& position)
{
	const auto velocity = Helper::GetRobotVelocity(robot).Length();
	const auto stopTime = velocity / Constants::Rules.ROBOT_ACCELERATION;
	const auto stopDist = velocity * stopTime - Constants::Rules.ROBOT_ACCELERATION * stopTime * stopTime / 2;

	auto deltaPos = position - Helper::GetRobotPosition(robot);

	Vector3D targetVelocity;
	if (stopDist < deltaPos.Length())
	{
		deltaPos.Normalize();
		targetVelocity = deltaPos * Constants::Rules.ROBOT_MAX_GROUND_SPEED;

	}
	else
	{
		deltaPos.Normalize();
		targetVelocity = deltaPos * (-Constants::Rules.ROBOT_MAX_GROUND_SPEED);
	}
	return targetVelocity;
}

std::optional<double> MyStrategy::GetOppStrikeTime(const Ball& ball, const std::vector<model::Robot>& oppRobots)
{
	BallEntity ball_entity = BallEntity(ball);
	auto nearest_robot = get_nearest_ball_robot(ball_entity, oppRobots);
	auto nearest_robot_entity = RobotEntity(nearest_robot);

	std::optional<double> collisionT;

	if (!nearest_robot_entity.Touch)
	{
		collisionT = Simulator::GetCollisionT(
			Helper::GetRobotPosition(nearest_robot),
			Helper::GetRobotVelocity(nearest_robot),
			Helper::GetBallPosition(ball),
			Helper::GetBallVelocity(ball));
		return collisionT;
	}

	
	if (IsOkOppPosToJump(ball_entity, nearest_robot_entity, collisionT))
	{		
		return collisionT;
	}

	//TODO: может прыгнуть позже, тогда итоговое время станет меньше (т.к. будет время на разгон)
	for( auto t = 1; t <= BallMoveTicks; ++t)
	{
		ball_entity = _ballEntities[t];		
		nearest_robot = get_nearest_ball_robot(ball_entity, oppRobots);		

		std::optional<double> jumpCollisionT = std::nullopt;
		std::optional<Vector3D> bestBallVelocity = std::nullopt;
		auto movePoint = GetAttackerStrikePoint(nearest_robot, t, -1, jumpCollisionT, bestBallVelocity);
		if (movePoint != std::nullopt)
		{
			collisionT = t * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();
			return collisionT;
		}

	}
	return std::nullopt;
}

model::Robot MyStrategy::get_nearest_ball_robot(const BallEntity& ball_entity, const std::vector<model::Robot>& oppRobots)
{
	auto min_dist = std::numeric_limits<double>::max();
	Robot nearest_robot{};
	for (Robot r : oppRobots)
	{
		auto const dist = Helper::GetLength2(ball_entity.Position, Helper::GetRobotPosition(r));
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest_robot = r;
		}	

	}
	return nearest_robot;

}

void MyStrategy::InitBallEntities(const model::Ball& ball)
{
	_ballEntities = std::map<int, BallEntity>();
	auto ball_entity = BallEntity(ball);
	bool isGoalScored = false;
	for (auto t = 1; t <= BallMoveTicks; ++t)
	{		
		ball_entity = SimulateTickBall(ball_entity, isGoalScored);
		_ballEntities[t] = ball_entity;
	}
}

int MyStrategy::UpdateBallEntities(double collisionTime, const Vector3D& afterCollisionBallVelocity)
{
	auto const preCollisionTick = int(collisionTime * Constants::Rules.TICKS_PER_SECOND);
	auto const afterCollisionTick = preCollisionTick + 1;
	auto ballEntity = _ballEntities[preCollisionTick];
	const auto beforeCollisionDeltaTime = collisionTime - preCollisionTick * 1.0 / Constants::Rules.TICKS_PER_SECOND;
	bool isGoalScored;
	Simulator::Update(ballEntity, beforeCollisionDeltaTime, isGoalScored);

	const auto afterCollisionDeltaTime = afterCollisionTick * 1.0 / Constants::Rules.TICKS_PER_SECOND - collisionTime;
	ballEntity.Velocity = Vector3D(afterCollisionBallVelocity);
	Simulator::Update(ballEntity, afterCollisionDeltaTime, isGoalScored);

	_ballEntities[afterCollisionTick] = ballEntity;
	for (auto i = 1; i < BallMoveTicks; ++i)
	{
		ballEntity =  SimulateTickBall(ballEntity, isGoalScored);
		_ballEntities[afterCollisionTick + i] = ballEntity;
		if (_goalScoringTick == -1 && ballEntity.Position.Z > Constants::Rules.arena.depth / 2 + Constants::Rules.BALL_RADIUS)
			_goalScoringTick = afterCollisionTick + i;
	}

	return afterCollisionTick;
}

std::string MyStrategy::custom_rendering()
{	
	std::string res = "";
	res += "[";
	for (int i =0 ; i < _drawSpheres.size(); ++i)
	{
		auto ds = _drawSpheres[i];
		res += "{";
		res += "\"Sphere\":{";
		res += "\"x\":" + std::to_string(ds.x) + ",";
		res += "\"y\":" + std::to_string(ds.y) + ",";
		res += "\"z\":" + std::to_string(ds.z) + ",";
		res += "\"radius\":" + std::to_string(ds.radius) + ",";
		res += "\"r\":" + std::to_string(ds.r) + ",";
		res += "\"g\":" + std::to_string(ds.g) + ",";
		res += "\"b\":" + std::to_string(ds.b) + ",";
		res += "\"a\":" + std::to_string(ds.a);
		
		res += "}";
		res += "}";
		if (i < _drawSpheres.size() - 1) res += ",";
	}
	res += "]";
	return res;
}