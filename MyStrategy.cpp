#include "MyStrategy.h"
#include "Simulator.h"
#include "Helper.h"
#include <cmath>
#include <sstream>
#include <algorithm>
#include <set>

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

	std::map<int, std::optional<double>> collisionTimes = std::map<int, std::optional<double>>();
	std::map<int, Vector3D> bestBallVelocities = std::map<int, Vector3D>();//TODO:если робот в воздухе - тут фигня
	if (_isFirstRobot)
	{
		_ball = game.ball;
		_robots = game.robots;
		_oppStrikeTime = std::nullopt;
		_drawSpheres = std::vector<Sphere>();
		InitBallEntities(game.ball, game.robots, collisionTimes, bestBallVelocities);
		_actions = std::map<int, model::Action > ();

		std::vector<Robot> opp_robots = std::vector<Robot>();
		for (Robot robot : game.robots)
		{
			if (!robot.is_teammate) opp_robots.push_back(robot);
		}
		if (!_oppStrikeTime.has_value())//м.б. установлено в InitBallEntities
			_oppStrikeTime = GetOppStrikeTime(opp_robots);
		_goalScoringTick = -1;
	}
	else
	{
		InitAction(action, me.id);
		return;
	}
	

	
	auto const ballEntity = BallEntity(game.ball);
	auto const isMeGoalPossible = IsGoalBallDirection2(ballEntity, -1, 0.0);

	std::vector <Robot> myRobots = std::vector<Robot>();
	Robot defender{};
	std::vector<Robot> attackers = std::vector<Robot>();
	bool isJumping = false;

	int maxCollisionTick = -1;
	for (Robot robot : game.robots)
	{
		if (!robot.is_teammate) continue;
		myRobots.push_back(robot);
		if (!robot.touch)
		{
			isJumping = true;
			InitJumpingRobotAction(robot, game.ball);
			if (collisionTimes[robot.id].has_value())//может и промахнуться
			{
				const auto collisionTick = int(collisionTimes[robot.id].value() * Constants::Rules.TICKS_PER_SECOND);
				if (collisionTick > maxCollisionTick)
					maxCollisionTick = collisionTick;
			}
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
			attackers.push_back(robot);
	}

	bool isDefender = false;
	if (isJumping)
	{
		if (isMeGoalPossible)
		{
			//защищаем ворота, начиная с 1 тика
			for (auto & robot : myRobots)
			{
				if (!robot.touch) continue;
				std::optional<double> collisionT = std::nullopt;
				auto bestBallVelocity = Helper::GetBallVelocity(_ball);
				Action robotAction = SetDefenderAction(
					robot, robot.id==defender.id ? _myGates : _beforeMyGates, isMeGoalPossible, collisionT, bestBallVelocity);
				collisionTimes[robot.id] = collisionT;
				bestBallVelocities[robot.id] = bestBallVelocity;
				_actions[robot.id] = robotAction;
			}
		}
		else
		{
			//идем в атаку, начиная с последнего тика коллизии прыгающего
			for (auto & robot : myRobots)
			{
				if (!robot.touch) continue;
				std::optional<double> collisionT = std::nullopt;
				auto bestBallVelocity = Helper::GetBallVelocity(_ball);
				Action robotAction = SetAttackerAction(
					robot, isMeGoalPossible, maxCollisionTick + AttackerAddTicks,
					robot.id == defender.id ? _myGates : _beforeMyGates, collisionT, bestBallVelocity, isDefender);
				collisionTimes[robot.id] = collisionT;
				bestBallVelocities[robot.id] = bestBallVelocity;
				_actions[robot.id] = robotAction;
			}
		}

		//bool isCollisionTime = false;
		//int bestCollisionTimeId = -1;
		//
		//for (auto item:collisionTimes)
		//{
		//	if (item.second != std::nullopt)
		//	{
		//		isCollisionTime = true;
		//		if (bestCollisionTimeId == -1) bestCollisionTimeId = item.first;
		//		if (_oppStrikeTime == std::nullopt || item.second.value() < _oppStrikeTime.value())
		//		{
		//			bestCollisionTimeId = item.first;
		//		};
		//	}				
		//}

		//if (bestCollisionTimeId >= 0 || isCollisionTime && !defender.touch)
		//{
		//	for (auto robot:myRobots)
		//	{
		//		if (!robot.touch) continue;

		//		Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
		//		std::optional<double> collisionT = std::nullopt;
		//		const Action robotAction = SetAttackerAction(
		//			robot, game.ball, isMeGoalPossible, 0 + AttackerAddTicks, //TODO: время начала атаки
		//			robot.id == defender.id ? _myGates : _beforeMyGates,
		//			collisionT, bestBallVelocity, isDefender);
		//		collisionTimes[robot.id] = collisionT;
		//		bestBallVelocities[robot.id] = bestBallVelocity;
		//		_actions[robot.id] = robotAction;
		//	}
		//}
		//else if (isCollisionTime)
		//{
		//	for (auto robot:myRobots)
		//	{
		//		if (!robot.touch) continue;
		//		Action robotAction = GetDefaultAction(robot, robot.id == defender.id ? _myGates : _beforeMyGates);
		//		collisionTimes[robot.id] = std::nullopt;
		//		bestBallVelocities[robot.id] = Helper::GetBallVelocity(game.ball);
		//		_actions[robot.id] = robotAction;
		//	}			
		//}
		//else
		//{
		//	if (defender.touch)
		//	{
		//		Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
		//		std::optional<double> collisionT = std::nullopt;
		//		Action robotAction = SetDefenderAction(defender, game.ball, _myGates, isMeGoalPossible, collisionT, bestBallVelocity);
		//		collisionTimes[defender.id] = collisionT;
		//		bestBallVelocities[defender.id] = bestBallVelocity;
		//		_actions[defender.id] = robotAction;
		//	}
		//	for (auto attacker:attackers)
		//	{
		//		if (!attacker.touch) continue;

		//		Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
		//		std::optional<double> collisionT = std::nullopt;
		//		Action robotAction = SetAttackerAction(attacker, game.ball, isMeGoalPossible, 1, _beforeMyGates, collisionT, bestBallVelocity, isDefender);
		//		collisionTimes[attacker.id] = collisionT;
		//		bestBallVelocities[attacker.id] = bestBallVelocity;
		//		_actions[attacker.id] = robotAction;
		//	}
		//}
	}
	else
	{		
		for (auto & attacker : attackers) //ищем точку, из которой нап может пробить по воротам противника
		{
			Vector3D attBestBallVelocity = Helper::GetBallVelocity(game.ball);
			std::optional<double> attCollisionT = std::nullopt;

			const Action attRobotAction = SetAttackerAction(
				attacker, isMeGoalPossible, 1, _beforeMyGates, attCollisionT, attBestBallVelocity, isDefender);
			collisionTimes[attacker.id] = attCollisionT;
			bestBallVelocities[attacker.id] = attBestBallVelocity;
			_actions[attacker.id] = attRobotAction;
		}

		if (!isDefender)//нашли точку для атаки ворот. защитник идет на ворота
		{
			Action robotAction = GetDefaultAction(defender, _myGates);
			collisionTimes[defender.id] = std::nullopt;
			bestBallVelocities[defender.id] = Helper::GetBallVelocity(game.ball);
			_actions[defender.id] = robotAction;
		}
		else //не нашли точку для атаки ворот.
		{
			Vector3D bestBallVelocity = Helper::GetBallVelocity(game.ball);
			std::optional<double> collisionT = std::nullopt;
			Action robotAction = SetDefenderAction(
				defender, _myGates, isMeGoalPossible, collisionT, bestBallVelocity);
			collisionTimes[defender.id] = collisionT;
			bestBallVelocities[defender.id] = bestBallVelocity;
			_actions[defender.id] = robotAction;

			//нашли точку для защитника, в которой он опередет напа врага
			if (collisionT != std::nullopt && (_oppStrikeTime == std::nullopt || collisionT.value() < _oppStrikeTime.value()))
			{
				const auto afterCollisionTick = UpdateBallEntities(
					collisionTimes[defender.id].value(), bestBallVelocities[defender.id]);
				//наши напы ориентируются на обновленную траекторию
				for (auto & attacker : attackers)
				{
					Vector3D attBestBallVelocity = Helper::GetBallVelocity(game.ball);
					std::optional<double> attCollisionT = std::nullopt;
					const Action attRobotAction = SetAttackerAction(
						attacker, isMeGoalPossible, afterCollisionTick + AttackerAddTicks, _beforeMyGates, attCollisionT, attBestBallVelocity, isDefender);
					collisionTimes[attacker.id] = attCollisionT;
					bestBallVelocities[attacker.id] = attBestBallVelocity;
					_actions[attacker.id] = attRobotAction;
				}
			} 
			else//защитник не опередит врага
			{
				int bestAttCollisionTId = -1;

				for (auto & attacker : attackers) //пытаемся опередить врага напом 
				{
					Vector3D attBestBallVelocity = Helper::GetBallVelocity(game.ball);
					std::optional<double> attCollisionT = std::nullopt;
					const Action attRobotAction = SetAttackerAction(
						attacker, isMeGoalPossible, 1, _beforeMyGates, attCollisionT, attBestBallVelocity, isDefender);

					if (attCollisionT != std::nullopt && (bestAttCollisionTId == -1 || attCollisionT.value() < collisionTimes[bestAttCollisionTId].value()))
					{
						bestAttCollisionTId = attacker.id;
					}

					collisionTimes[attacker.id] = attCollisionT;
					bestBallVelocities[attacker.id] = attBestBallVelocity;
					_actions[attacker.id] = attRobotAction;
				}

				//нашли точку для напа, в которой он опередет напа врага
				if (bestAttCollisionTId >= 0 &&
					(_oppStrikeTime == std::nullopt || collisionTimes[bestAttCollisionTId].value() < _oppStrikeTime.value()))
				{
					//защитник идет в атаку
					//TODO: он перестает идти в опр. момент

					const auto afterCollisionTick = UpdateBallEntities(
						collisionTimes[bestAttCollisionTId].value(), bestBallVelocities[bestAttCollisionTId]);

					Vector3D defBestBallVelocity = Helper::GetBallVelocity(game.ball);
					std::optional<double> defCollisionT = std::nullopt;
					//TODO: защитник должен возвращаться на _gates, а не _beforeGates
					const Action defRobotAction = SetAttackerAction(
						defender, isMeGoalPossible, afterCollisionTick + AttackerAddTicks, _myGates, defCollisionT, defBestBallVelocity, isDefender);
					collisionTimes[defender.id] = defCollisionT;
					bestBallVelocities[defender.id] = defBestBallVelocity;
					_actions[defender.id] = defRobotAction;
				}
				else //нап не опередит врага
				{
					//будем выбивать мяч как можно раньше. не факт, что враг прыгнет
					//выбиваем оптимальный Action. Action другого робота сбрасываем на дефолтный
					for (Robot & robot : myRobots)
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

void MyStrategy::InitJumpingRobotAction(const Robot& robot, const Ball& ball)
{
	Action robotAction = model::Action();

	RobotEntity re = RobotEntity(robot);
	model::Action reAction = Action();
	reAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
	re.Action = reAction;

	Simulator::Tick(re, BallEntity(ball));
	if (!re.IsArenaCollided) robotAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
	_actions[robot.id] = robotAction;

	//collisionTimes, bestBallVelocities установятся в InitBallEntities
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

BallEntity MyStrategy::SimulateTickBall(
	const BallEntity & ballEntity, std::vector<RobotEntity>& jumpRes, bool & isGoalScored, bool discardIsCollided) const
{
	BallEntity bec = BallEntity(ballEntity);
	std::vector<RobotEntity> jumpResC = std::vector<RobotEntity>();
	for (const auto& jr : jumpRes)
		jumpResC.push_back(RobotEntity(jr));

	Simulator::Update(bec, jumpResC, 1.0 / Constants::Rules.TICKS_PER_SECOND, isGoalScored);
	if (!bec.IsArenaCollided && !bec.IsCollided)
	{	

		bool isJrCollided = false;
		for (const auto& jr : jumpResC)
		{
			if (jr.IsCollided)
			{
				isJrCollided = true;
				break;
			}
		}

		if (!isJrCollided)
		{
			jumpRes = jumpResC;
			return bec;
		}
	}

	bec = BallEntity(ballEntity);

	Simulator::Tick(bec, jumpRes);
	bec.IsArenaCollided = false;	
	bec.IsCollided = false;

	if (discardIsCollided)
		for (auto& jr : jumpRes)
			jr.IsCollided = false;
	return bec;
}


bool MyStrategy::SimulateCollision(BallEntity & ballEntity, RobotEntity & robotEntity,
	std::optional<double>& collisionT, int beforeTicks) 
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
	const auto afterJumpCollisionT = Simulator::GetCollisionT(robotEntity.Position, robotEntity.Velocity, ballEntity.Position,
		ballEntity.Velocity);
	if (afterJumpCollisionT == std::nullopt) return false;

	collisionT = afterJumpCollisionT.value() + 2.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;

	return SimulateNoTouchEntitiesCollision(
		ballEntity,
		robotEntity,
		afterJumpCollisionT.value(),
		collisionT.value(),
		beforeTicks);
	
}

bool MyStrategy::SimulateNoTouchEntitiesCollision(
	BallEntity & ballEntity, RobotEntity & robotEntity, double robotCollisionT, double ballCollisionT, int beforeTicks)
{
	robotEntity.IsArenaCollided = false;

	//ситуаци¤ за микротик до коллизии.	
	auto const collisionMicroTicks = int(robotCollisionT * Constants::Rules.TICKS_PER_SECOND * Constants::Rules.MICROTICKS_PER_TICK);
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

	const auto collisionTick = beforeTicks + int(ballCollisionT * Constants::Rules.TICKS_PER_SECOND);
	if (_ballEntities.count(collisionTick) == 0)
		return false;//TODO: тик коллизии не симмулирован

	const auto collisionTickBe = _ballEntities.at(collisionTick);
	const auto timeLeft = ballCollisionT -
		int(ballCollisionT * Constants::Rules.TICKS_PER_SECOND) * 1.0 / Constants::Rules.TICKS_PER_SECOND;
		
	auto const mtLeft = int(timeLeft * Constants::Rules.TICKS_PER_SECOND * Constants::Rules.MICROTICKS_PER_TICK);
	auto const mtLeftTime = mtLeft * 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;

	bool needBeSimulation = false;
	const auto bePosition = Vector3D(
		collisionTickBe.Position.X + collisionTickBe.Velocity.X * mtLeftTime,
		collisionTickBe.Position.Y + collisionTickBe.Velocity.Y * mtLeftTime - 
			Constants::Rules.GRAVITY * mtLeftTime * mtLeftTime / 2,
		collisionTickBe.Position.Z + collisionTickBe.Velocity.Z * mtLeftTime);
	dan = DanCalculator::GetDanToArena(bePosition, Constants::Rules.arena);
	if (ballEntity.Radius - dan.Distance > 0) needBeSimulation = true;

	//TODO:мяч может "пролететь" через стену (например, опускаясь на ворота)
	if (!needBeSimulation)
	{
		ballEntity.Position = bePosition;
		ballEntity.Velocity = Vector3D(
			collisionTickBe.Velocity.X, 
			collisionTickBe.Velocity.Y - Constants::Rules.GRAVITY * mtLeftTime, 
			collisionTickBe.Velocity.Z);
		return true;
	}	

	//TODO: Неточно, т.к. может быть коллизи¤ м¤ча с ареной или коллизи¤ робота с ареной
	ballEntity = BallEntity(collisionTickBe);
	std::vector<RobotEntity> res = std::vector<RobotEntity>();//предполагаем, что за столь малое время коллизий с роботами не будет
	bool isGoalScored;
	Simulator::Update(ballEntity,
		res,
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

bool MyStrategy::IsGoalBallDirection2(const BallEntity & startBallEntity, int directionCoeff, double ballEntityTime)
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

	

	bool isGoalScored = false;	

	const auto tick = int(ballEntityTime * Constants::Rules.TICKS_PER_SECOND);
	const auto nextTick = tick + 1;
	const auto nextTickTime = nextTick * 1.0 / Constants::Rules.TICKS_PER_SECOND;
	const auto deltaTime = nextTickTime - ballEntityTime;

	BallEntity ballEntity = BallEntity(startBallEntity);
	ballEntity.Position.X += ballEntity.Velocity.X * deltaTime;
	ballEntity.Position.Y += ballEntity.Velocity.Y * deltaTime - Constants::Rules.GRAVITY * deltaTime * deltaTime / 2.0;
	ballEntity.Position.Z += ballEntity.Velocity.Z * deltaTime;
	double z = ballEntity.Position.Z;

	std::vector<RobotEntity> jumpResCur = std::vector<RobotEntity>();
	if (_robotEntities.count(nextTick) > 0)
		for (auto & re : _robotEntities.at(nextTick))
		{
			jumpResCur.push_back(RobotEntity(re));
		}

	for (int t = 1; t <= BallMoveTicks; ++t)
	{
		ballEntity = SimulateTickBall(ballEntity, jumpResCur, isGoalScored, true);
		if (isGoalScored) return true;
		const auto isZIncreasing = ballEntity.Position.Z * directionCoeff > z * directionCoeff;		
		if (!isZIncreasing) return false;
		z = ballEntity.Position.Z;

		jumpResCur.erase(
			std::remove_if(
				jumpResCur.begin(), jumpResCur.end(), [](RobotEntity re) {return re.IsArenaCollided; }), jumpResCur.end());
		
	}	
	return false;
}

model::Action MyStrategy::SetDefenderAction(
	const model::Robot & me, const Vector3D & defenderPoint,
	bool isMeGoalPossible,
	std::optional<double>& collisionT, Vector3D& bestBallVelocity)
{
	model::Action action = model::Action();
	Vector3D targetVelocity;
	
	std::optional<Vector3D> defendPoint =
		GetDefenderMovePoint(me, isMeGoalPossible, collisionT, bestBallVelocity);

	std::optional<double> jumpCollisionT = std::nullopt;
	std::optional<Vector3D> jumpBallVelocity = std::nullopt;

	if (IsOkDefenderPosToJump(
		Helper::GetRobotPosition(me),
		Helper::GetRobotVelocity(me),
		isMeGoalPossible,
		true,
		0,
		jumpCollisionT,
		jumpBallVelocity))
	{
		if (defendPoint == std::nullopt || CompareBallVelocities(jumpBallVelocity.value(), bestBallVelocity) < 0)
		{
			collisionT = jumpCollisionT;
			bestBallVelocity = jumpBallVelocity.value();

			targetVelocity =
				Helper::GetTargetVelocity(me.x, 0, me.z, _ball.x, 0, _ball.z, Constants::Rules.ROBOT_MAX_GROUND_SPEED);
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
	const BallEntity ballEntity = _ballEntities.at(t);

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

		if (_ballEntities.at(moveT).Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)//м¤ч в воротах
			return bestBallVelocity;

		std::optional<double> curJumpCollisionT = std::nullopt;
		std::optional<Vector3D> collisionBallVelocity = std::nullopt;

		if (IsOkDefenderPosToJump(pvContainer.Position, pvContainer.Velocity,
			isMeGoalPossible,
			isDefender,
			moveT,
			curJumpCollisionT, collisionBallVelocity))
		{			

			if (CompareBallVelocities(collisionBallVelocity.value(), bestBallVelocity) < 0)
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
	bool isMeGoalPossible,
	bool isDefender,
	int beforeTicks,
	std::optional<double>& jumpCollisionT, std::optional<Vector3D>& collisionBallVelocity)
{
	auto moveTBallEntity = BallEntity(_ballEntities.at(beforeTicks));
	if (moveTBallEntity.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
	{
		return false;
	}

	auto robotEntity = RobotEntity(
		Vector3D(robotPosition), Vector3D(robotVelocity), Constants::Rules.ROBOT_MIN_RADIUS,
		true, Vector3D(0, 1, 0), 0.0);

	std::optional<double> jumpCollisionTCur = std::nullopt;
	auto const isCollision = SimulateCollision(moveTBallEntity, robotEntity, jumpCollisionTCur, beforeTicks);
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
			const auto beTime = beforeTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionTCur.value();
			const auto isCollisionGoalPossible = IsGoalBallDirection2(ballEntity2, -1, beTime);
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

std::optional<Vector3D> MyStrategy::GetDefenderMovePoint(const model::Robot & robot,
	bool isMeGoalPossible,
	std::optional<double>& collisionT, Vector3D& bestBallVelocity)
{
	std::optional<Vector3D> movePoint = std::nullopt;

	int bestT = -1;

	for (int t = 1; t <= BallMoveTicks; ++t)
	{
		auto ballEntity = _ballEntities.at(t);
		
		std::optional<double> curCollisionT = std::nullopt;
		bool isPassedBy = false;
		std::optional<Vector3D> ballVelocity = 
			GetDefenderStrikeBallVelocity(robot, t, 1, isMeGoalPossible, true, curCollisionT, isPassedBy);

		if (ballVelocity == std::nullopt) continue;

		const auto tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
		if (_oppStrikeTime.has_value() && collisionT.has_value())
		{
			if (collisionT.value() > _oppStrikeTime.value() - tickTime)
			{
				if (curCollisionT.value() >= collisionT.value())
					continue;
				else
				{
					bestBallVelocity = ballVelocity.value();
					collisionT = curCollisionT.value();
					const auto be = _ballEntities.at(t);
					movePoint = Vector3D(be.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
						be.Position.Z);
					bestT = t;
					continue;
				}
			}
			else
			{
				if (curCollisionT.value() >= _oppStrikeTime.value() - tickTime)
					continue;
			}
		}

		if (CompareBallVelocities(*ballVelocity, bestBallVelocity) < 0)
		{			
			bestBallVelocity = ballVelocity.value();
			collisionT = curCollisionT.value();
			const auto be = _ballEntities.at(t);
			movePoint = Vector3D(be.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
				be.Position.Z);
			bestT = t;
		}
	}

	if(movePoint != std::nullopt)
	{
		_drawSpheres.push_back(Sphere((*movePoint).X, (*movePoint).Y, (*movePoint).Z, 0.25, 0, 0, 1, 0.5));
		const auto be = _ballEntities.at(bestT);
		_drawSpheres.push_back(Sphere(
			be.Position.X, be.Position.Y, be.Position.Z,
			Constants::Rules.BALL_RADIUS, 0, 0, 1, 0));
	}
	
	return movePoint;
}

model::Action MyStrategy::SetAttackerAction(const model::Robot & me, 
	bool isMeGoalPossible,
	int startAttackTick,
	const Vector3D& defenderPoint,
	std::optional<double>& collisionT, Vector3D& bestBallVelocity, bool& isDefender)
{
	model::Action action = model::Action();

	Vector3D targetVelocity;
	RobotEntity robotEntity = RobotEntity(me);

	std::optional<double> jumpCollisionT = std::nullopt;
	std::optional<Vector3D> jumpBallVelocity = std::nullopt;

	if (startAttackTick == 1 && IsOkPosToJump(robotEntity, 0, jumpCollisionT, jumpBallVelocity))
	{
		isDefender = false;
		collisionT = jumpCollisionT;
		bestBallVelocity = jumpBallVelocity.value();

		targetVelocity =
			Helper::GetTargetVelocity(me.x, 0, me.z, _ball.x, 0, _ball.z, Constants::Rules.ROBOT_MAX_GROUND_SPEED);
		action.target_velocity_x = targetVelocity.X;
		action.target_velocity_y = 0.0;
		action.target_velocity_z = targetVelocity.Z;
		action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
		action.use_nitro = false;
		return action;
	}

	std::optional<Vector3D> movePoint = 
		GetAttackerMovePoint(me, isMeGoalPossible, startAttackTick, collisionT, isDefender, bestBallVelocity);
	if (isDefender)
	{
		jumpCollisionT = std::nullopt;
		jumpBallVelocity = std::nullopt;

		auto const isGoalPossible = IsGoalBallDirection2(_ballEntities.at(0), -1 , 0);
		if (startAttackTick == 1 && IsOkDefenderPosToJump(
			Helper::GetRobotPosition(me),
			Helper::GetRobotVelocity(me),
			isGoalPossible,
			false,
			0,
			jumpCollisionT, jumpBallVelocity))
		{
			if (movePoint == std::nullopt || CompareBallVelocities(jumpBallVelocity.value(), bestBallVelocity) < 0)
			{
				collisionT = jumpCollisionT;
				bestBallVelocity = jumpBallVelocity.value();

				targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, _ball.x, 0, _ball.z,
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
				Helper::GetLength2(Helper::GetBallPosition(_ball), Helper::GetRobotPosition(me)) >
				_distToFollowBall * _distToFollowBall
				&&	me.z < _ball.z)
			{
				movePoint = Vector3D(_ball.x, 0, _ball.z);
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

bool MyStrategy::IsOkPosToMove(const Vector3D & mePos, const model::Robot & robot,
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

	if (Helper::GetLength(_ballEntities.at(t).Position, pvContainer.Position) < 
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
		IsOkPosToJump(robotE, t, jumpCollisionT, ballVelocity) :
		IsOkOppPosToJump(robotE, t, jumpCollisionT);

	if (isOkPosToStrike)
	{
		if (directionCoeff == 1)
		{
			const auto color = 0;
			_drawSpheres.push_back(Sphere(mePos.X, mePos.Y, mePos.Z, 0.25, color, color, color, 0.5));
			_drawSpheres.push_back(Sphere(pvContainer.Position.X, pvContainer.Position.Y, pvContainer.Position.Z, 1,
				color, color, color, 0.5));
			auto const be = _ballEntities.at(t);
			_drawSpheres.push_back(Sphere(be.Position.X, be.Position.Y, be.Position.Z,
				Constants::Rules.BALL_RADIUS, 0, 1, 0, 0));
		}
		collisionT = jumpCollisionT;
		bestBallVelocity = ballVelocity;
		return true;
	}

	return false;
}

std::optional<Vector3D> MyStrategy::GetAttackerMovePoint(const model::Robot & robot,
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
		if (_ballEntities.count(t) == 0) return movePoint;

		auto ballEntity = _ballEntities.at(t);		

		if (IsPenaltyArea(ballEntity.Position, false))//включаем режим защитника
		{
			isDefender = true;
			std::optional<double> curCollisionT = std::nullopt;
			bool isPassedBy = false;
			std::optional<Vector3D> ballVelocity = 
				GetDefenderStrikeBallVelocity(robot, t, startAttackTick, isMeGoalPossible, false, curCollisionT, isPassedBy);
			if (ballVelocity == std::nullopt) continue;

			const auto tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
			if (_oppStrikeTime.has_value() && collisionT.has_value())
			{
				if (collisionT.value() > _oppStrikeTime.value() - tickTime)
				{
					if (curCollisionT.value() >= collisionT.value())
						continue;
					else
					{
						bestBallVelocity = ballVelocity.value();
						collisionT = curCollisionT.value();
						const auto be = _ballEntities.at(t);

						movePoint = Vector3D(be.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
							be.Position.Z);
						bestT = t;
						continue;
					}
				}
				else
				{
					if (curCollisionT.value() >= _oppStrikeTime.value() - tickTime)
						continue;
				}
			}
			
			if (CompareBallVelocities(ballVelocity.value(), bestBallVelocity) < 0)
			{
				bestBallVelocity = ballVelocity.value();
				collisionT = curCollisionT.value();
				const auto be = _ballEntities.at(t);

				movePoint = Vector3D(be.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
					be.Position.Z);
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
		const auto be = _ballEntities.at(bestT);
		_drawSpheres.push_back(Sphere(
			be.Position.X, be.Position.Y, be.Position.Z,
			Constants::Rules.BALL_RADIUS, 1, 0, 0, 0));
	}

	return movePoint;
}

std::optional<Vector3D> MyStrategy::GetAttackerStrikePoint(const model::Robot & robot, int t,
	int directionCoeff,
	std::optional<double>& collisionT, std::optional<Vector3D>& bestBallVelocity)
{
	const BallEntity ballEntity = _ballEntities.at(t);
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
	if (IsOkPosToMove(mePos, robot, t, directionCoeff, collisionT, bestBallVelocity))
	{
		_beforeStrikePoints[robot.id] = mePos;
		return mePos;
	}

	for (int i = 1; i <= StrikeSphereStepsCount; ++i)
	{
		Vector3D rVectorMult1 = rVector1 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos1 = Vector3D(ballEntity.Position.X + rVectorMult1.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult1.Z);
		if (IsOkPosToMove(mePos1, robot, t, directionCoeff, collisionT, bestBallVelocity))
		{
			_beforeStrikePoints[robot.id] = mePos1;
			return mePos1;
		}

		Vector3D rVectorMult2 = rVector2 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos2 = Vector3D(ballEntity.Position.X + rVectorMult2.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult2.Z);
		if (IsOkPosToMove(mePos2, robot, t, directionCoeff, collisionT, bestBallVelocity))
		{
			_beforeStrikePoints[robot.id] = mePos2;
			return mePos2;
		}
	}

	return std::nullopt;
}

bool MyStrategy::IsOkPosToJump(
	RobotEntity & robotEntity,
	int beforeTicks,
	std::optional<double>& collisionT, std::optional<Vector3D>& bestBallVelocity)
{
	const auto directionCoeff = 1;

	if (_ballEntities.at(beforeTicks).Velocity.Z <=0 && 
		Helper::GetLength2(robotEntity.Position, _ballEntities.at(beforeTicks).Position) > _maxStrikeDist2 + EPS)
	{
		collisionT = std::nullopt;
		return false;
	}

	//auto const prevBallEntity = BallEntity(ballEntity);

	std::optional<double> jumpCollisionT = std::nullopt;
	auto ballEntity = BallEntity(_ballEntities.at(beforeTicks));
	auto const isCollision = SimulateCollision(ballEntity, robotEntity, jumpCollisionT, beforeTicks);
	if (!isCollision) return false;

	if(robotEntity.Velocity.Y < 0) return false;//не бьем на излете

	//коллизи¤ в штрафной площади
	if (IsPenaltyArea(ballEntity.Position, false))
		return false;

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

		const auto beTime = beforeTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();

		if (!IsGoalBallDirection2(ballEntity2, directionCoeff, beTime)) return false;
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

bool MyStrategy::IsOkOppPosToJump(
	RobotEntity & robotEntity, int beforeTicks, std::optional<double>& collisionT)
{
	const auto directionCoeff = -1;
	//auto const prevBallEntity = BallEntity(ballEntity);

	auto ballEntity = BallEntity(_ballEntities.at(beforeTicks));
	std::optional<double> jumpCollisionT = std::nullopt;
	const auto isCollision = SimulateCollision(ballEntity, robotEntity, jumpCollisionT, beforeTicks);
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

		const auto beTime = beforeTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();
		if (!IsGoalBallDirection2(ballEntity2, directionCoeff, beTime)) return false;
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

std::optional<double> MyStrategy::GetOppStrikeTime(const std::vector<model::Robot>& oppRobots)
{
	auto nearest_robot = get_nearest_ball_robot(_ballEntities.at(0), oppRobots);
	auto nearest_robot_entity = RobotEntity(nearest_robot);

	std::optional<double> collisionT;

	if (!nearest_robot_entity.Touch)
	{
		/*collisionT = Simulator::GetCollisionT(
			Helper::GetRobotPosition(nearest_robot),
			Helper::GetRobotVelocity(nearest_robot),
			Helper::GetBallPosition(ball),
			Helper::GetBallVelocity(ball));
		return collisionT;*/
		return std::nullopt; //удар летящего врага уже учтен в траектории
	}

	
	if (IsOkOppPosToJump(nearest_robot_entity, 0, collisionT))
	{		
		return collisionT;
	}

	//TODO: может прыгнуть позже, тогда итоговое время станет меньше (т.к. будет время на разгон)
	for( auto t = 1; t <= BallMoveTicks; ++t)
	{
		nearest_robot = get_nearest_ball_robot(_ballEntities.at(t), oppRobots);

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

void MyStrategy::InitBallEntities(
	const model::Ball& ball, const std::vector<Robot>& robots,
	std::map<int, std::optional<double>>& collisionTimes,
	std::map<int, Vector3D>& bestBallVelocities)
{
	bool isGoalScored = false;
	_ballEntities = std::map<int, BallEntity>();	
	auto ball_entity = BallEntity(ball);
	_ballEntities[0] = ball_entity;

	_robotEntities = std::map<int, std::vector<RobotEntity>>();
	_robotEntities[0] = std::vector<RobotEntity>();
	for (auto & robot : _robots)
	{
		if (!robot.touch)
		{
			auto re = RobotEntity(robot);
			re.Action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
			_robotEntities[0].push_back(re);
		}
	}	
	
	
	for (auto t = 1; t <= BallMoveTicks; ++t)
	{
		std::vector<RobotEntity> jumpResCur = std::vector<RobotEntity>();
		for (auto & re: _robotEntities.at(t-1))
		{
			jumpResCur.push_back(RobotEntity(re));
		}

		ball_entity = SimulateTickBall(ball_entity, jumpResCur, isGoalScored, false);	

		_ballEntities[t] = ball_entity;

		for (auto& re : jumpResCur)
		{
			if (re.IsCollided)
			{
				bool isTeammate = false;
				for (auto & r : robots)
				{
					if (r.id == re.Id)
					{
						isTeammate = r.is_teammate;
						break;
					}
				}

				if (isTeammate)
				{
					collisionTimes[re.Id] = t * 1.0 / Constants::Rules.TICKS_PER_SECOND;
				}
				re.IsCollided = false;
			}
		}

		jumpResCur.erase(
			std::remove_if(
				jumpResCur.begin(), jumpResCur.end(), [](RobotEntity re) {return re.IsArenaCollided; }), jumpResCur.end());

		_robotEntities[t] = jumpResCur;		

	}

	/*std::set<int> gotIds = std::set<int>();
	auto const ballVelocity = Helper::GetBallVelocity(ball);

	while (true)
	{
		auto minCollisionT = std::numeric_limits<double>::max();
		auto needCalc = false;
		Robot bestRobot{};
		for (auto robot : robots)
		{
			if (robot.touch) continue;

			if (gotIds.find(robot.id) != gotIds.end()) continue;
			gotIds.insert(robot.id);

			auto collisionT = Simulator::GetCollisionT(
				Helper::GetRobotPosition(robot),
				Helper::GetRobotVelocity(robot),
				Helper::GetBallPosition(ball),
				Helper::GetBallVelocity(ball));
			if (robot.is_teammate)
			{
				collisionTimes[robot.id] = collisionT;
				bestBallVelocities[robot.id] = ballVelocity;
			}

			if (!collisionT.has_value()) continue;
			if (!robot.is_teammate) _oppStrikeTime = collisionT;

			needCalc = true;
			if (collisionT.value() < minCollisionT)
			{
				minCollisionT = collisionT.value();
				bestRobot = robot;
			}
		}

		if (!needCalc) break;

		auto collisionBe = BallEntity(ball);
		auto collisionRe = RobotEntity(bestRobot);
		collisionRe.Action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
		const auto isCollision = SimulateNoTouchEntitiesCollision(collisionBe, collisionRe, minCollisionT, minCollisionT, 0);
		if (isCollision)
		{
			const auto hitE = (_hitEs[0] + _hitEs[1]) / 2;
			Simulator::Update(collisionRe, collisionBe,
				1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK, hitE, isGoalScored);
			if (collisionBe.IsCollided)
			{
				if (bestRobot.is_teammate)
					bestBallVelocities[bestRobot.id] = collisionBe.Velocity;

				UpdateBallEntities(minCollisionT, collisionBe.Velocity, ball);
			}
		}
		else if (bestRobot.is_teammate)
		{
			collisionTimes[bestRobot.id] = std::nullopt;
			bestBallVelocities[bestRobot.id] = ballVelocity;
		}
	}*/

	for (auto t = 1; t <= BallMoveTicks; ++t)
	{
		const auto be = _ballEntities.at(t);
		_drawSpheres.push_back(Sphere(
			be.Position.X,
			be.Position.Y,
			be.Position.Z,
			2, 1, 1, 0, 0.5
		));
		for (auto & re:_robotEntities.at(t))
		{
			_drawSpheres.push_back(Sphere(
				re.Position.X,
				re.Position.Y,
				re.Position.Z,
				1, 1, 0.5, 0, 0.5
			));
		}
		
	}
}

int MyStrategy::UpdateBallEntities(double collisionTime, const Vector3D& afterCollisionBallVelocity)
{
	auto const preCollisionTick = int(collisionTime * Constants::Rules.TICKS_PER_SECOND);
	auto const afterCollisionTick = preCollisionTick + 1;
	auto ballEntity = BallEntity(_ballEntities.at(preCollisionTick));
	auto robotEntities = std::vector<RobotEntity>();
	for (const auto & re : _robotEntities.at(preCollisionTick))
		robotEntities.push_back(RobotEntity(re));

	const auto beforeCollisionDeltaTime = collisionTime - preCollisionTick * 1.0 / Constants::Rules.TICKS_PER_SECOND;
	bool isGoalScored;

	Simulator::Update(ballEntity, robotEntities, beforeCollisionDeltaTime, isGoalScored);

	const auto afterCollisionDeltaTime = afterCollisionTick * 1.0 / Constants::Rules.TICKS_PER_SECOND - collisionTime;
	ballEntity.Velocity = Vector3D(afterCollisionBallVelocity);
	Simulator::Update(ballEntity, robotEntities, afterCollisionDeltaTime, isGoalScored);

	_ballEntities[afterCollisionTick] = ballEntity;
	_robotEntities[afterCollisionTick] = robotEntities;

	for (auto i = 1; i < BallMoveTicks; ++i)
	{
		std::vector<RobotEntity> jumpResCur = std::vector<RobotEntity>();
		for (auto & re : _robotEntities.at(afterCollisionTick + i - 1))
		{
			jumpResCur.push_back(RobotEntity(re));
		}

		ballEntity =  SimulateTickBall(ballEntity, jumpResCur, isGoalScored, true);
		_ballEntities[afterCollisionTick + i] = ballEntity;

		jumpResCur.erase(
			std::remove_if(
				jumpResCur.begin(), jumpResCur.end(), [](RobotEntity re) {return re.IsArenaCollided; }), jumpResCur.end());
		_robotEntities[afterCollisionTick + i] = jumpResCur;

		if (_goalScoringTick == -1 && ballEntity.Position.Z > Constants::Rules.arena.depth / 2 + Constants::Rules.BALL_RADIUS)
			_goalScoringTick = afterCollisionTick + i;

		const auto be = _ballEntities.at(afterCollisionTick + i);
		_drawSpheres.push_back(Sphere(
			be.Position.X,
			be.Position.Y,
			be.Position.Z,
			2, 0, 1, 1, 0.25
		));
		for (auto & re : _robotEntities.at(afterCollisionTick + i))
		{
			_drawSpheres.push_back(Sphere(
				re.Position.X,
				re.Position.Y,
				re.Position.Z,
				1, 0, 0.5, 1, 0.25
			));
		}
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
		res += "},";
		//if (i < _drawSpheres.size() - 1) res += ",";
	}
	res += "{\"Text\": \"" + (_oppStrikeTime.has_value() ? std::to_string(*_oppStrikeTime * 60) : "-1") + "\"}";
	res += "]";
	return res;
}