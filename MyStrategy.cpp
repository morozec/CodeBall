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
	std::map<int, BallEntity> bestBallEntities = std::map<int, BallEntity>();//TODO:если робот в воздухе - тут фигня
	if (_isFirstRobot)
	{
		_isGoalPossible = false;
		_isMeGoalPossible = false;
		_isNoCollisionGoalPossible = false;
		_isNoCollisionMeGoalPossible = false;

		_ball = game.ball;
		_robots = game.robots;
		_oppStrikeTime = std::nullopt;
		_drawSpheres = std::vector<Sphere>();
		InitBallEntities(collisionTimes, bestBallEntities);
		_actions = std::map<int, model::Action > ();

		std::vector<Robot> opp_robots = std::vector<Robot>();
		for (Robot robot : game.robots)
		{
			if (!robot.is_teammate) opp_robots.push_back(robot);
		}
		if (!_oppStrikeTime.has_value())//м.б. установлено в InitBallEntities
			_oppStrikeTime = GetOppStrikeTime(opp_robots);
		_goalScoringTick = -1;
		_meGoalScoringTick = -1;
	}
	else
	{
		InitAction(action, me.id);
		return;
	}
	

	
	auto const ballEntity = BallEntity(game.ball);

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

	//bool isDefender = false;
	//bool isStrikeDefender = false;
	if (isJumping)
	{
		//if (_isMeGoalPossible)
		//{
		//	//защищаем ворота, начиная с 1 тика
		//	for (auto & robot : myRobots)
		//	{
		//		if (!robot.touch) continue;
		//		std::optional<double> collisionT = std::nullopt;
		//		auto best_ball_entity = BallEntity(_ball);
		//		Action robotAction = SetDefenderAction(
		//			robot, robot.id==defender.id ? _myGates : _beforeMyGates, collisionT, best_ball_entity);
		//		collisionTimes[robot.id] = collisionT;
		//		bestBallEntities[robot.id] = best_ball_entity;
		//		_actions[robot.id] = robotAction;
		//	}
		//}
		//else
		//{
		//	//идем в атаку, начиная с последнего тика коллизии прыгающего
		//	for (auto & robot : myRobots)
		//	{
		//		if (!robot.touch) continue;
		//		std::optional<double> collisionT = std::nullopt;
		//		auto bestBallEntity = BallEntity(_ball);
		//		Action robotAction = SetAttackerAction(
		//			robot, maxCollisionTick == -1 ? 1 : maxCollisionTick + AttackerAddTicks,
		//			robot.id == defender.id ? _myGates : _beforeMyGates, collisionT, bestBallEntity, isDefender, isStrikeDefender);
		//		collisionTimes[robot.id] = collisionT;
		//		bestBallEntities[robot.id] = bestBallEntity;
		//		_actions[robot.id] = robotAction;
		//	}
		//}

		//идем в атаку, начиная с последнего тика коллизии прыгающего
		for (auto & robot : myRobots)
		{
			if (!robot.touch) continue;
			BallEntityContainer bec;
			bool isDefender;
			bool isOkBestBec;
			const Action robotAction = SetAttackerAction(
				robot, maxCollisionTick == -1 ? 1 : maxCollisionTick + AttackerAddTicks,
				robot.id == defender.id ? _myGates : _beforeMyGates, bec, isDefender, isOkBestBec);
			_actions[robot.id] = robotAction;
		}
		
	}
	else
	{		
		int bestBecPRobotId = -1;
		BallEntityContainer bestBec;
		for (auto& robot:myRobots)
		{
			if (!robot.touch) continue;
			bool isDefender;
			BallEntityContainer curBestBec;
			bool isOkBestBec;
			const auto attAction = SetAttackerAction(robot, 1, robot.id == defender.id ? _myGates : _beforeMyGates, curBestBec, isDefender, isOkBestBec);
			if (isOkBestBec)
			{
				if (bestBecPRobotId == -1 || CompareBeContainers(curBestBec, bestBec) < 0)
				{
					bestBec = curBestBec;
					bestBecPRobotId = robot.id;
				}
			}
			_actions[robot.id] = attAction;
		}

		if (bestBecPRobotId != -1)
			for (auto & robot:myRobots)
			{
				if (!robot.touch) continue;
				if (robot.id == bestBecPRobotId) continue;

				/*if (_defenderMovePoints.count(robot.id) > 0)
					_defenderMovePoints.erase(robot.id);*/

				if (bestBec.ResBallEntity.Position.Z < 0 && (!_oppStrikeTime.has_value() || bestBec.collisionTime < _oppStrikeTime.value()))//бьем со своей половины - второй идет добивать
				{
					const auto collisionTime = bestBec.collisionTime;
					const auto afterCollisionTick = UpdateBallEntities(collisionTime, bestBec.ResBallEntity.Velocity);

					bool isDefender;
					BallEntityContainer curBestBec;
					bool isOkBestBec;
					const Action attAction = SetAttackerAction(
						robot, afterCollisionTick + AttackerAddTicks, robot.id == defender.id ? _myGates : _beforeMyGates, curBestBec, isDefender, isOkBestBec);
					_actions[robot.id] = attAction;
				}
				else //бьем с чужой половины - второй идет на ворота
				{
					const Action defAction = GetDefaultAction(robot, _myGates);
					_actions[robot.id] =defAction;
				}
				
			}




	//	int strikeDefenderId = -1;
	//	int bestAttCollisionTId = -1;
	//	
	//	auto gotStriker = false;
	//	for (auto & attacker : attackers) //ищем точку, из которой нап может пробить по воротам противника
	//	{
	//		BallEntity attBestBallEntity = BallEntity(game.ball);
	//		std::optional<double> attCollisionT = std::nullopt;

	//		const Action attRobotAction = SetAttackerAction(
	//			attacker, 1, _beforeMyGates, attCollisionT, attBestBallEntity, isDefender, isStrikeDefender);

	//		if (attCollisionT != std::nullopt && (bestAttCollisionTId == -1 || attCollisionT.value() < collisionTimes[bestAttCollisionTId].value()))
	//		{
	//			bestAttCollisionTId = attacker.id;
	//		}
	//		if (!isDefender && attCollisionT != std::nullopt)
	//			gotStriker = true;

	//		collisionTimes[attacker.id] = attCollisionT;
	//		bestBallEntities[attacker.id] = attBestBallEntity;
	//		_actions[attacker.id] = attRobotAction;
	//		if (isStrikeDefender)
	//			strikeDefenderId = attacker.id;
	//	}
	//	if (isStrikeDefender)//нап бьет по воротам из зоны защиты. защитнику можно идти вперед
	//	{
	//		const auto afterCollisionTick = UpdateBallEntities(
	//			collisionTimes[strikeDefenderId].value(), bestBallEntities[strikeDefenderId].Velocity);

	//		auto defBestBallEntity = BallEntity(game.ball);
	//		std::optional<double> defCollisionT = std::nullopt;
	//		const Action defRobotAction = SetAttackerAction(
	//			defender, afterCollisionTick + AttackerAddTicks, _myGates, defCollisionT, defBestBallEntity, isDefender, isStrikeDefender);
	//		collisionTimes[defender.id] = defCollisionT;
	//		bestBallEntities[defender.id] = defBestBallEntity;
	//		_actions[defender.id] = defRobotAction;
	//	}

	//	else if (gotStriker)//нашли точку для атаки ворот. защитник идет на ворота
	//	{
	//		Action robotAction = GetDefaultAction(defender, _myGates);
	//		collisionTimes[defender.id] = std::nullopt;
	//		bestBallEntities[defender.id] = BallEntity(game.ball);
	//		_actions[defender.id] = robotAction;
	//	}
	//	else //не нашли точку для атаки ворот.
	//	{
	//		auto bestBallentity = BallEntity(game.ball);
	//		std::optional<double> collisionT = std::nullopt;
	//		Action robotAction = SetDefenderAction(
	//			defender, _myGates, collisionT, bestBallentity);
	//		collisionTimes[defender.id] = collisionT;
	//		bestBallEntities[defender.id] = bestBallentity;
	//		_actions[defender.id] = robotAction;

	//		//нашли точку для защитника, в которой он опередет напа врага
	//		if (collisionT != std::nullopt && (_oppStrikeTime == std::nullopt || collisionT.value() < _oppStrikeTime.value()))
	//		{
	//			const auto afterCollisionTick = UpdateBallEntities(
	//				collisionTimes[defender.id].value(), bestBallEntities[defender.id].Velocity);
	//			//наши напы ориентируются на обновленную траекторию
	//			for (auto & attacker : attackers)
	//			{
	//				auto attBestBallEntity = BallEntity(game.ball);
	//				std::optional<double> attCollisionT = std::nullopt;
	//				const Action attRobotAction = SetAttackerAction(
	//					attacker, afterCollisionTick + AttackerAddTicks, _beforeMyGates, attCollisionT, attBestBallEntity, isDefender, isStrikeDefender);
	//				collisionTimes[attacker.id] = attCollisionT;
	//				bestBallEntities[attacker.id] = attBestBallEntity;
	//				_actions[attacker.id] = attRobotAction;
	//			}
	//		} 
	//		else//защитник не опередит врага
	//		{					
	//			//нашли точку для напа, в которой он опередет напа врага
	//			if (bestAttCollisionTId >= 0 &&
	//				(_oppStrikeTime == std::nullopt || collisionTimes[bestAttCollisionTId].value() < _oppStrikeTime.value()))
	//			{
	//				//защитник идет в атаку
	//				//TODO: он перестает идти в опр. момент

	//				const auto afterCollisionTick = UpdateBallEntities(
	//					collisionTimes[bestAttCollisionTId].value(), bestBallEntities[bestAttCollisionTId].Velocity);

	//				auto defBestBallEntity = BallEntity(game.ball);
	//				std::optional<double> defCollisionT = std::nullopt;
	//				const Action defRobotAction = SetAttackerAction(
	//					defender, afterCollisionTick + AttackerAddTicks, _myGates, defCollisionT, defBestBallEntity, isDefender, isStrikeDefender);
	//				collisionTimes[defender.id] = defCollisionT;
	//				bestBallEntities[defender.id] = defBestBallEntity;
	//				_actions[defender.id] = defRobotAction;
	//			}
	//			else //нап не опередит врага
	//			{
	//				//будем выбивать мяч как можно раньше. не факт, что враг прыгнет
	//				//выбиваем оптимальный Action. Action другого робота сбрасываем на дефолтный
	//				for (Robot & robot : myRobots)
	//				{
	//					if (!robot.touch) continue;

	//					Robot otherRobot{};
	//					for (Robot r : game.robots)
	//					{
	//						if (!r.is_teammate || r.id == robot.id) continue;
	//						otherRobot = r;
	//						break;
	//					}

	//					if (collisionTimes[robot.id] != std::nullopt && collisionTimes[otherRobot.id] != std::nullopt)
	//					{
	//						if (_oppStrikeTime == std::nullopt ||
	//							std::max(*collisionTimes[robot.id], *collisionTimes[otherRobot.id]) < _oppStrikeTime.value())
	//						{
	//							int isB2GoalDirection = -1;
	//							if (!otherRobot.touch ||  //если один уже в воздухе, оптимизация не имеет смысла
	//								CompareDefenderBallEntities(bestBallEntities[robot.id], bestBallEntities[otherRobot.id], isB2GoalDirection) > 0)
	//							{
	//								_actions[robot.id] =
	//									GetDefaultAction(robot, robot.z < otherRobot.z ? _myGates : _beforeMyGates);
	//							}
	//						}

	//						else if (*collisionTimes[robot.id] > *collisionTimes[otherRobot.id])
	//						{
	//							_actions[robot.id] =
	//								GetDefaultAction(robot, robot.z < otherRobot.z ? _myGates : _beforeMyGates);
	//						}
	//					}
	//				}
	//			}

	//		}
	//	}
	}		
	
	InitAction(action, me.id);
	for (auto & bsp:_beforeStrikePoints)
	{
		_beforeStrikePoints[bsp.first].first--;
	}

	std::vector<int> removeIds = std::vector<int>();
	for (auto & dmp : _defenderMovePoints)
	{
		_defenderMovePoints[dmp.first].first--;
		_defenderMovePoints[dmp.first].second--;
		if (_defenderMovePoints[dmp.first].second <  0 || _defenderMovePoints[dmp.first].first < 0)
		{
			removeIds.push_back(dmp.first);
		}
	}
	for (auto removeId : removeIds)
		_defenderMovePoints.erase(removeId);
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

	_allHitEs[0]= (Constants::Rules.MAX_HIT_E + Constants::Rules.MIN_HIT_E) / 2.0;
	_allHitEs[1]= Constants::Rules.MIN_HIT_E;
	_allHitEs[2]= Constants::Rules.MAX_HIT_E;

	_averageHitE[0] = (Constants::Rules.MAX_HIT_E + Constants::Rules.MIN_HIT_E) / 2.0;
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
	const auto targetVelocity = GetDefendPointTargetVelocity(me, defaultPos);
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
		jumpResC.push_back(jr);

	const double hitE = (_hitEs[0] + _hitEs[1]) / 2.0;
	Simulator::UpdateOnAir(bec, jumpResC, 1.0 / Constants::Rules.TICKS_PER_SECOND, hitE, isGoalScored, true);
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

	Simulator::Tick(bec, jumpRes, hitE);
	

	if (discardIsCollided)
	{
		bec.IsArenaCollided = false;
		bec.IsCollided = false;
		for (auto& jr : jumpRes)
			jr.IsCollided = false;
	}
	return bec;
}


bool MyStrategy::SimulateCollision(BallEntity & ballEntity, RobotEntity& robotEntity,
                                   const double hitEs[], int hitEsSize,
									std::vector<BallEntity>& resBes, std::vector<double>& resCollisionTimes,
									int beforeTicks, bool forbidNegativeVy, bool forbidDownStrike)
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
	/*Simulator::Update(robotEntity, ballEntity,
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
	Simulator::Update(robotEntity, ballEntity,
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
	*/

	//считаем время коллизии
		
	Simulator::simulate_jump_start(robotEntity);
	robotEntity.IsArenaCollided = false;

	const auto twoMtTime = 2.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;
	ballEntity.Position.X += ballEntity.Velocity.X * twoMtTime;
	ballEntity.Position.Y += ballEntity.Velocity.Y * twoMtTime - 
		Constants::Rules.GRAVITY * twoMtTime * twoMtTime / 2.0;
	ballEntity.Position.Z += ballEntity.Velocity.Z * twoMtTime;
	ballEntity.Velocity.Y -= Constants::Rules.GRAVITY * twoMtTime;


	auto jumpRes = std::vector<RobotEntity>();
	jumpRes.push_back(robotEntity);

	for (const auto & re : _robotEntities.at(beforeTicks))
	{
		auto reCopy = RobotEntity(re);
		reCopy.Position.X += reCopy.Velocity.X * twoMtTime;
		reCopy.Position.Y += reCopy.Velocity.Y * twoMtTime -
			Constants::Rules.GRAVITY * twoMtTime * twoMtTime / 2.0;
		reCopy.Position.Z += reCopy.Velocity.Z * twoMtTime;
		reCopy.Velocity.Y -= Constants::Rules.GRAVITY * twoMtTime;
		jumpRes.push_back(reCopy);
	}	


	resBes = std::vector<BallEntity>();
	resCollisionTimes = std::vector<double>();

	for (int i = 0; i < hitEsSize; ++i)
	{
		auto const hitE = hitEs[i];
		auto beCopy = BallEntity(ballEntity);
		auto jumpResCopy = std::vector<RobotEntity>();
		for (auto const & re: jumpRes)
		{
			jumpResCopy.push_back(re);
		}
		double afterJumpCollisionT = 0;
		auto const isCol = SimulateFullCollision(beCopy, jumpResCopy, afterJumpCollisionT, hitE, forbidNegativeVy, forbidDownStrike);
		if (!isCol)
			return false;

		resBes.push_back(beCopy);
		resCollisionTimes.push_back(afterJumpCollisionT + 2.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK);
	}

	return true;
	
	/*collisionT = afterJumpCollisionT.value() + twoMtsTime;

	return SimulateNoTouchEntitiesCollision(
		ballEntity,
		robotEntity,
		afterJumpCollisionT.value(),
		collisionT.value(),
		beforeTicks);	*/
}

bool MyStrategy::SimulateFullCollision(
	BallEntity & be, std::vector<RobotEntity>& res, double& collisionT, double hitE, bool forbidNegativeVy, bool forbidDownStrike) const
{
	bool isGoalScored;
	collisionT = 0;
	bool myRobotExists = true;
	while (true)
	{
		auto minCollisionT = std::numeric_limits<double>::max();
		Entity* e1 = nullptr;
		Entity* e2 = nullptr;

		for (auto & re : res)
		{
			auto curCollisionT = Simulator::GetCollisionT(
				re.Position, re.Velocity, be.Position, be.Velocity, re.Radius, be.Radius);
			if (curCollisionT == std::nullopt) continue;

			//наработки по учету реального положения мяча
			/*auto fullCollisionTime = beforeTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK + addTime + curCollisionT.value();
			auto fullCollisionTicks = int(fullCollisionTime * Constants::Rules.TICKS_PER_SECOND * Constants::Rules.MICROTICKS_PER_TICK);
			if (_ballEntities.count(fullCollisionTime) == 0) continue;
			auto fctBe = BallEntity(_ballEntities.at(fullCollisionTime));
			auto deltaTime = fullCollisionTime - fullCollisionTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;
			fctBe.Position.X += fctBe.Velocity.X * deltaTime;
			fctBe.Position.Y += fctBe.Velocity.Y * deltaTime - Constants::Rules.GRAVITY * deltaTime * deltaTime / 2;
			fctBe.Position.Z += fctBe.Velocity.Z * deltaTime;

			curCollisionT = Simulator::GetCollisionT(
				re.Position, re.Velocity, fctBe.Position, fctBe.Velocity, re.Radius, fctBe.Radius);
			if (curCollisionT == std::nullopt) continue;*/

			if (curCollisionT.value() < minCollisionT)
			{
				minCollisionT = curCollisionT.value();
				e1 = &re;
				e2 = &be;
			}
		}

		for (auto i = 0; i < res.size(); ++i)
		{
			auto reI = res.at(i);
			for (int j = 0; j < i; ++j)
			{
				auto reJ = res.at(j);
				const auto curCollisionT = Simulator::GetCollisionT(
					reI.Position, reI.Velocity, reJ.Position, reJ.Velocity, reI.Radius, reJ.Radius);
				if (curCollisionT == std::nullopt) continue;
				if (curCollisionT.value() < minCollisionT)
				{
					minCollisionT = curCollisionT.value();
					e1 = &reI;
					e2 = &reJ;
				}
			}			
		}
		if (e1 == nullptr && e2 == nullptr)
		{			
			if (myRobotExists && !res.at(0).IsCollided) return false;//полет был бесполезен
			return true;
		}

		const auto minCollisionMicroTicks = int(
			minCollisionT * Constants::Rules.TICKS_PER_SECOND * Constants::Rules.MICROTICKS_PER_TICK);
		const auto beforCollisionTime =
			minCollisionMicroTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;

		auto beCopy = BallEntity(be);
		Simulator::UpdateOnAir(beCopy, res, beforCollisionTime, hitE, isGoalScored, true);

		if (myRobotExists)
		{
			if (res.at(0).IsArenaCollided)
			{
				if (!res.at(0).IsCollided)//врезались в арену до того, как врезаться в мяч/робота
				{
					return false;
				}
				else //уже в кого-то предварительно врезались. просто исключаем этого робота
				{
					myRobotExists = false;
				}
			}				
		}

		if (beCopy.IsArenaCollided)
		{
			const bool isHorMove = abs(beCopy.Position.Y - Constants::Rules.BALL_RADIUS) < EPS && abs(beCopy.Velocity.Y) < 0.01;
			if (!isHorMove)
				return !myRobotExists || res.at(0).IsCollided;
		}

		//убираем тех, кто врезался в арену
		res.erase(
			std::remove_if(
				res.begin(), res.end(), [](RobotEntity re) {return re.IsArenaCollided; }), res.end());

		if (res.empty())
		{
			return true;
		}	
		collisionT += minCollisionT;

		if (e2 == &be) 
		{
			if (myRobotExists && e1 == &res.at(0) && !res.at(0).IsCollided)
			{				
				//if (e1 != &res.at(0))
				//	return false;//коллизия с мячом произойдет до моего вмешательства
				if (forbidNegativeVy && res.at(0).Velocity.Y < 0)
					return false;
				if (forbidDownStrike && res.at(0).Position.Y > beCopy.Position.Y)
					return false;
			}						

			//return true;//выходим за микротик до коллизии с мячом
		}
		be = beCopy;
		Simulator::UpdateOnAir(
			be, res, 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
			hitE, isGoalScored, false);
	}
	throw "WHY AM I HERE?";
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

//-1 - не определено, 0 - нет, 1 - да
int MyStrategy::CompareDefenderBallEntities(const BallEntityContainer & b1, const BallEntityContainer & b2) const
{
	if (b1.ResBallEntity.Velocity.Z > 0 && b2.ResBallEntity.Velocity.Z > 0)
	{
		//return b1.ResBallEntity.Velocity.Z > b2.ResBallEntity.Velocity.Z ? -1 : 1;

		const double v1HorAngle = GetVectorAngleToHorizontal(b1.ResBallEntity.Velocity);
		const double v2HorAngle = GetVectorAngleToHorizontal(b1.ResBallEntity.Velocity);
		if (v1HorAngle > M_PI / 9 && v2HorAngle > M_PI / 9)
		{
			return b1.ResBallEntity.Velocity.Z > b2.ResBallEntity.Velocity.Z ? -1 : 1;
		}
		return b1.ResBallEntity.Velocity.Y > b2.ResBallEntity.Velocity.Y ? -1 : 1;
	}

	auto b1Vz = b1.ResBallEntity.Velocity.Z;
	if (b1Vz < 0) b1Vz = b1.changeVelocityZ;

	auto b2Vz = b2.ResBallEntity.Velocity.Z;
	if (b2Vz < 0) b2Vz = b2.changeVelocityZ;

	return b1Vz > b2Vz ? -1 : 1;

	/*const double v1HorAngle = GetVectorAngleToHorizontal(b1.ResBallEntity.Velocity);
	const double v2HorAngle = GetVectorAngleToHorizontal(b1.ResBallEntity.Velocity);
	if (v1HorAngle > M_PI / 9 && v2HorAngle > M_PI / 9)
	{
		return b1Vz > b2Vz ? -1 : 1;
	}
	return b1.ResBallEntity.Velocity.Y > b2.ResBallEntity.Velocity.Y ? -1 : 1;*/
}

int MyStrategy::CompareBeContainers(BallEntityContainer bec1, BallEntityContainer bec2) const
{
	const auto tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
	if (_oppStrikeTime.has_value())
	{
		if (bec2.collisionTime > _oppStrikeTime.value() - tickTime)
		{
			return bec1.collisionTime < bec2.collisionTime ? -1 : 1;
		}
		else
		{
			if (bec1.collisionTime >= _oppStrikeTime.value() - tickTime)
				return 1;
		}
	}

	if (bec2.isGoalScored && !bec1.isGoalScored)
		return 1;
	if (bec1.isGoalScored && !bec2.isGoalScored)
		return -1;
	if (bec1.isGoalScored && bec2.isGoalScored)
	{
		if (bec1.ResBallEntity.Position.Z > 0 || bec2.ResBallEntity.Position.Z > 0)//чужой половине бьем быстрее
		{
			return bec1.GetFullGoalTime() < bec2.GetFullGoalTime() ? -1 : 1;
		}

		return  bec1.ResBallEntity.Velocity.Y > bec2.ResBallEntity.Velocity.Y ? -1 : 1;
	}

	//оба не isGoalScored	
	return CompareDefenderBallEntities(bec1, bec2);
}

bool MyStrategy::IsGoalBallDirection2(const BallEntity & startBallEntity, int directionCoeff, bool considerBoardSide , double& goalTime, double& changeDirVz) const
{
	if (startBallEntity.Velocity.Z * directionCoeff <= 0) return false;
	if (abs(startBallEntity.Velocity.Z) < EPS) return false;
	
	const auto ballVelocityZDist = abs(startBallEntity.Velocity.Z) * BallMoveTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND;
	const auto ballToGatesDist = abs(startBallEntity.Position.Z - (Constants::Rules.arena.depth / 2 + startBallEntity.Radius) * directionCoeff);
	if (ballVelocityZDist < ballToGatesDist) return false;//мяч не докатится до ворот за 100 тиков
	
	if (!considerBoardSide)
	{

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
	}

	bool isGoalScored = false;	
	BallEntity ballEntity = BallEntity(startBallEntity);
	double z = ballEntity.Position.Z;
	std::vector<RobotEntity> emptyJumpRes = std::vector<RobotEntity>();

	int collisions = 0;
	for (int t = 1; t <= BallMoveTicks; ++t)
	{		
		ballEntity = SimulateTickBall(ballEntity, emptyJumpRes, isGoalScored, false);
		if (isGoalScored) {
			goalTime = t * 1.0 / Constants::Rules.TICKS_PER_SECOND;
			return true;
		}

		if (ballEntity.IsArenaCollided)
		{
			changeDirVz = ballEntity.Velocity.Z;
		}

		if (directionCoeff == 1 && ballEntity.IsArenaCollided)
		{
			collisions++;
			if (collisions > MaxCollisionsToGoalStrike)
				return false;			
		}
		ballEntity.IsArenaCollided = false;

		const auto isZIncreasing = ballEntity.Position.Z * directionCoeff > z * directionCoeff;		
		if (!isZIncreasing) 
		{			
			return false;
		}
		z = ballEntity.Position.Z;
		
	}	
	return false;
}

//model::Action MyStrategy::SetDefenderAction(
//	const model::Robot & me, const Vector3D & defenderPoint,
//	std::optional<double>& collisionT, BallEntity& bestBallEntity)
//{
//	model::Action action = model::Action();
//	Vector3D targetVelocity;
//	std::optional<double> jumpCollisionT = std::nullopt;
//	std::optional<BallEntity> jump_ball_entity = std::nullopt;
//
//	if (_defenderMovePoints.count(me.id) > 0)
//	{
//		const auto moveTime = _defenderMovePoints[me.id].second;
//		if (moveTime == 0)
//		{			
//			if (IsOkDefenderPosToJump(
//				Helper::GetRobotPosition(me),
//				Helper::GetRobotVelocity(me),
//				true,
//				0,
//				jumpCollisionT,
//				jump_ball_entity))
//			{
//				collisionT = jumpCollisionT;
//				bestBallEntity = jump_ball_entity.value();
//
//				targetVelocity =
//					Helper::GetTargetVelocity(me.x, 0, me.z, _ball.x, 0, _ball.z, Constants::Rules.ROBOT_MAX_GROUND_SPEED);
//				action.target_velocity_x = targetVelocity.X;
//				action.target_velocity_y = 0.0;
//				action.target_velocity_z = targetVelocity.Z;
//				action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
//				action.use_nitro = false;
//
//				for (auto it = _defenderMovePoints.cbegin(); it != _defenderMovePoints.cend();)//удаляем все остальные схраненки
//				{
//					if ((*it).first != me.id) 
//						_defenderMovePoints.erase(it);
//					++it;
//				}
//
//				return action;
//			}
//			_defenderMovePoints.erase(me.id);
//		}		
//	}
//
//
//	bool isSavedPointOk;
//	std::optional<Vector3D> defendPoint =
//		GetDefenderMovePoint(me, collisionT, bestBallEntity, isSavedPointOk);	
//
//	if (!isSavedPointOk && IsOkDefenderPosToJump(
//		Helper::GetRobotPosition(me),
//		Helper::GetRobotVelocity(me),
//		true,
//		0,
//		jumpCollisionT,
//		jump_ball_entity))
//	{
//		int isB2GoalDirection = -1;
//		if (defendPoint == std::nullopt || CompareDefenderBallEntities(jump_ball_entity.value(), bestBallEntity, isB2GoalDirection) < 0)
//		{
//			collisionT = jumpCollisionT;
//			bestBallEntity = jump_ball_entity.value();
//
//			targetVelocity =
//				Helper::GetTargetVelocity(me.x, 0, me.z, _ball.x, 0, _ball.z, Constants::Rules.ROBOT_MAX_GROUND_SPEED);
//			action.target_velocity_x = targetVelocity.X;
//			action.target_velocity_y = 0.0;
//			action.target_velocity_z = targetVelocity.Z;
//			action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
//			action.use_nitro = false;
//
//			return action;
//		}
//	}
//
//
//	if (defendPoint == std::nullopt) {
//		targetVelocity = GetDefendPointTargetVelocity(me, defenderPoint);
//	}
//	else
//	{
//		targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, defendPoint.value().X, 0, defendPoint.value().Z,
//			Constants::Rules.ROBOT_MAX_GROUND_SPEED);
//	}
//
//	action.target_velocity_x = targetVelocity.X;
//	action.target_velocity_y = 0.0;
//	action.target_velocity_z = targetVelocity.Z;
//	action.jump_speed = 0.0;
//	action.use_nitro = false;
//	return action;
//}

bool MyStrategy::GetDefenderStrikeBallEntity(const model::Robot & robot, int t,
	int startAttackTick,///для сохраненных точек передаем t прыжка
	int endAttackTick, //для сохраненных точек передаем t прыжка + 1
	BallEntityContainer& bestBecP, int& bestMoveT)
{
	int isB2GoalDirection = -1;
	const BallEntity ballEntity = _ballEntities.at(t);
	bool isOk = false;

	Vector3D targetPos = Vector3D(ballEntity.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
		ballEntity.Position.Z);

	for (int moveT = startAttackTick; moveT < endAttackTick; ++moveT)
	{
		/*if (best_ball_velocity != std::nullopt &&
			_oppStrikeTime.has_value() && 
			moveT * 1.0 / Constants::Rules.TICKS_PER_SECOND > _oppStrikeTime.value())
			return;*/

		PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
			Helper::GetRobotPosition(robot),
			targetPos,
			Helper::GetRobotVelocity(robot),
			moveT,
			1.0);

		if (pvContainer.IsPassedBy)
		{
			return isOk; // проскочим целевую точку. дальше все непредсказуемо
		}

		if (_ballEntities.at(moveT).Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)//м¤ч в воротах
			return isOk;

		std::optional<double> curJumpCollisionT = std::nullopt;
		std::optional<BallEntity> collision_ball_entity = std::nullopt;
		double changeDirVz = 0;

		if (IsOkDefenderPosToJump(pvContainer.Position, pvContainer.Velocity,
			moveT,
			curJumpCollisionT, collision_ball_entity, changeDirVz))
		{	
			double goalTime=-1;
			double changeDirVzAttack = 0;
			const auto isGoal = IsGoalBallDirection2(collision_ball_entity.value(), 1, true, goalTime, changeDirVzAttack);
			const double curCollisionT = moveT * 1.0 / Constants::Rules.TICKS_PER_SECOND + curJumpCollisionT.value();
			const auto bec = BallEntityContainer(collision_ball_entity.value(), curCollisionT, isGoal, goalTime, changeDirVz);

			if (!isOk || CompareBeContainers(bec, bestBecP) < 0)
			{
				/*if (best_ball_velocity != std::nullopt &&
					_oppStrikeTime.has_value() &&
					curCollisionT > _oppStrikeTime.value())
					continue;*/
				isOk = true;
				bestBecP = bec;
				bestMoveT = moveT;
			}
		}
	}
	return isOk;	

}

bool MyStrategy::IsOkDefenderPosToJump(
	const Vector3D & robotPosition, const Vector3D & robotVelocity,  
	int beforeTicks,
	std::optional<double>& jumpCollisionT, std::optional<BallEntity>& collisionBallEntity, double& changeDirVz)
{
	auto moveTBallEntity = BallEntity(_ballEntities.at(beforeTicks));
	if (moveTBallEntity.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
	{
		return false;
	}

	auto robotEntity = RobotEntity(
		Vector3D(robotPosition), Vector3D(robotVelocity), Constants::Rules.ROBOT_MIN_RADIUS,
		true, Vector3D(0, 1, 0), 0.0);

	std::vector<BallEntity> resBes;
	std::vector<double> resCollisionTimes;
	auto const isCollision = SimulateCollision(moveTBallEntity, robotEntity, _allHitEs, 3, resBes, resCollisionTimes, beforeTicks, true, true);
	if (!isCollision) return false;

	double goalTime = -1;
	for (int i = 0; i < 3; ++i)
	{
		auto beCur = resBes.at(i);
		auto const jumpCollisionTCur = resCollisionTimes.at(i);		

		if (i == 0)
		{
			collisionBallEntity = BallEntity(beCur);
			collisionBallEntity->IsCollided = false;
			collisionBallEntity->IsArenaCollided = false;
			jumpCollisionT = jumpCollisionTCur;
		}

		if (!beCur.IsCollided)
		{
			if (_isNoCollisionMeGoalPossible) return false;//нет смысла бить по роботу - без роботов мяч летит в мои ворота
			if (!_isMeGoalPossible) return false; //нет смысла бить по роботу - сейчас мяч не летит в мои ворота
			continue;
		}
		beCur.IsCollided = false;
		beCur.IsArenaCollided = false;

		if (beCur.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
			return false;

		//коллизи¤ вне штрафной площади
		/*if (!IsPenaltyArea(beCur.Position, isDefender))
			return false;*/

		if (beCur.Velocity.Z < 0)
		{
			if (!_isMeGoalPossible) return false;
			double changeDirVzCur = 0;
			const auto isCollisionGoalPossible = IsGoalBallDirection2(beCur, -1, false, goalTime, changeDirVzCur);
			if (i==0)
			{
				changeDirVz = changeDirVzCur;
			}

			if (isCollisionGoalPossible) return false;
		}
	}	
	
	return true;
}

//std::optional<Vector3D> MyStrategy::GetDefenderMovePoint(const model::Robot & robot,
//	std::optional<double>& collisionT, BallEntity& bestBallEntity, bool& isSavedPointOk)
//{
//	isSavedPointOk = false;
//	const auto tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
//
//	if (_defenderMovePoints.count(robot.id) > 0)
//	{	
//		const auto tBall = _defenderMovePoints[robot.id].first;
//		const auto tJump = _defenderMovePoints[robot.id].second;
//
//		if (tBall > 0 && tJump > 0 && tBall <= BallMoveTicks) //tBall > BallMoveTicks может получиться, если нашли точку для атаки в SetAttAction со StartTick > 1
//		{
//			std::optional<double> curCollisionT = std::nullopt;
//			bool isPassedBy = false;
//			int bestMoveT;
//			std::optional<BallEntity> ball_entity =
//				GetDefenderStrikeBallEntity(robot, tBall, tJump, tJump + 1, true, curCollisionT, isPassedBy, bestMoveT);
//
//			if (ball_entity != std::nullopt && (!_oppStrikeTime.has_value() || curCollisionT.value() < _oppStrikeTime.value() - tickTime))
//			{
//				isSavedPointOk = true;
//				bestBallEntity = ball_entity.value();
//				collisionT = curCollisionT.value();
//				const auto be = _ballEntities.at(tBall);
//
//				for (auto it = _defenderMovePoints.cbegin(); it != _defenderMovePoints.cend();)//удаляем все остальные схраненки
//				{
//					if ((*it).first != robot.id)
//						_defenderMovePoints.erase(it);
//					++it;
//				}
//
//				return Vector3D(be.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
//					be.Position.Z);
//			}
//		}
//		_defenderMovePoints.erase(robot.id);		
//	}
//
//	std::optional<Vector3D> movePoint = std::nullopt;
//	int bestT = -1;
//	int isB2GoalDirection = -1;
//	double goalTime;
//
//	for (int t = 1; t <= BallMoveTicks; ++t)
//	{
//		auto ballEntity = _ballEntities.at(t);
//		
//		std::optional<double> curCollisionT = std::nullopt;
//		bool isPassedBy = false;
//		int bestMoveT;
//		std::optional<BallEntity> ball_entity = 
//			GetDefenderStrikeBallEntity(robot, t, 1, t, true, curCollisionT, isPassedBy, bestMoveT);
//
//		if (ball_entity == std::nullopt) continue;
//
//		
//		if (_oppStrikeTime.has_value() && collisionT.has_value())
//		{
//			if (collisionT.value() > _oppStrikeTime.value() - tickTime)
//			{
//				if (curCollisionT.value() >= collisionT.value())
//					continue;
//				else
//				{
//					bestBallEntity = ball_entity.value();
//					collisionT = curCollisionT.value();
//					const auto be = _ballEntities.at(t);
//					movePoint = Vector3D(be.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
//						be.Position.Z);
//					bestT = t;
//					if (IsGoalBallDirection2(bestBallEntity, 1, true, goalTime))
//					{
//						_defenderMovePoints[robot.id] = std::pair<int, int>();
//						_defenderMovePoints[robot.id].first = t;
//						_defenderMovePoints[robot.id].second = bestMoveT;
//					}
//					else
//					{
//						if (_defenderMovePoints.count(robot.id) > 0)
//							_defenderMovePoints.erase(robot.id);
//					}
//
//					continue;
//				}
//			}
//			else
//			{
//				if (curCollisionT.value() >= _oppStrikeTime.value() - tickTime)
//					continue;
//			}
//		}
//
//		if (CompareDefenderBallEntities(ball_entity.value(), bestBallEntity, isB2GoalDirection) < 0)
//		{
//			bestBallEntity = ball_entity.value();
//			collisionT = curCollisionT.value();
//			const auto be = _ballEntities.at(t);
//			movePoint = Vector3D(be.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
//				be.Position.Z);
//			bestT = t;
//			if (IsGoalBallDirection2(bestBallEntity, 1, true, goalTime))
//			{
//				_defenderMovePoints[robot.id] = std::pair<int, int>();
//				_defenderMovePoints[robot.id].first = t;
//				_defenderMovePoints[robot.id].second =bestMoveT;
//			}
//			else
//			{
//				if (_defenderMovePoints.count(robot.id) > 0)
//					_defenderMovePoints.erase(robot.id);
//			}
//		}
//	}
//
//	if(movePoint != std::nullopt)
//	{
//		_drawSpheres.push_back(Sphere((*movePoint).X, (*movePoint).Y, (*movePoint).Z, 0.25, 0, 0, 1, 0.5));
//		const auto be = _ballEntities.at(bestT);
//		_drawSpheres.push_back(Sphere(
//			be.Position.X, be.Position.Y, be.Position.Z,
//			Constants::Rules.BALL_RADIUS, 0, 0, 1, 0));
//	}
//	
//	return movePoint;
//}

model::Action MyStrategy::SetAttackerAction(const model::Robot & me, 
	int startAttackTick,
	const Vector3D& defenderPoint,
	BallEntityContainer& bestBecP,
	bool& isDefender, bool& isOkBestBecP)
{
	model::Action action = model::Action();

	Vector3D targetVelocity;
	RobotEntity robotEntity = RobotEntity(me);	

	bool isDefenderSavedPointOk;
	std::optional<Vector3D> movePoint =
		GetAttackerMovePoint(
			me, startAttackTick, isDefender, isDefenderSavedPointOk, bestBecP);

	std::optional<double> jumpCollisionT = std::nullopt;
	std::optional<BallEntity> jump_ball_entity = std::nullopt;
	double changeDirVz = 0;
	double goalTime = 0;
	if (startAttackTick == 1 && IsOkPosToJump(robotEntity, 0, jumpCollisionT, jump_ball_entity, goalTime))
	{
		auto jumpBec = BallEntityContainer(jump_ball_entity.value(), jumpCollisionT.value(), true, goalTime, 0);
		if (movePoint == std::nullopt || CompareBeContainers(jumpBec, bestBecP) < 0)
		{
			isOkBestBecP = true;
			isDefender = false;			
			bestBecP = jumpBec;

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

	if (startAttackTick == 1 && _defenderMovePoints.count(me.id) > 0)
	{
		const auto moveTime = _defenderMovePoints[me.id].second;
		if (moveTime == 0)
		{
			if (IsOkDefenderPosToJump(
				Helper::GetRobotPosition(me),
				Helper::GetRobotVelocity(me),
				0,
				jumpCollisionT,
				jump_ball_entity,
				changeDirVz))
			{
				isOkBestBecP = true;
				isDefender = true;
				double changeDirVzAttack = 0;
				auto const isGoal = IsGoalBallDirection2(jump_ball_entity.value(), 1, true, goalTime, changeDirVzAttack);
				//if (!isGoal) throw "NO GOAL FOR SAVED POINT"; м.б. коллизия
				
				const auto bec = BallEntityContainer(
					jump_ball_entity.value(), jumpCollisionT.value(), true, goalTime, changeDirVz);
				bestBecP = bec;

				targetVelocity =
					Helper::GetTargetVelocity(me.x, 0, me.z, _ball.x, 0, _ball.z, Constants::Rules.ROBOT_MAX_GROUND_SPEED);
				action.target_velocity_x = targetVelocity.X;
				action.target_velocity_y = 0.0;
				action.target_velocity_z = targetVelocity.Z;
				action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
				action.use_nitro = false;

				for (auto it = _defenderMovePoints.cbegin(); it != _defenderMovePoints.cend();)//удаляем все остальные схраненки
				{
					if ((*it).first != me.id)
						_defenderMovePoints.erase(it);
					++it;
				}

				return action;
			}
			_defenderMovePoints.erase(me.id);
		}
	}

	if (isDefender || movePoint == std::nullopt)
	{
		jumpCollisionT = std::nullopt;
		jump_ball_entity = std::nullopt;

		if (!isDefenderSavedPointOk && startAttackTick == 1 && IsOkDefenderPosToJump(
			Helper::GetRobotPosition(me),
			Helper::GetRobotVelocity(me),
			0,
			jumpCollisionT, jump_ball_entity, changeDirVz))
		{
			double changeDirVzAttack = 0;
			auto const isGoal = IsGoalBallDirection2(jump_ball_entity.value(), 1, true, goalTime, changeDirVzAttack);
			auto jumpBec = BallEntityContainer(jump_ball_entity.value(), jumpCollisionT.value(), isGoal, goalTime, changeDirVz);
			if (movePoint == std::nullopt || CompareBeContainers(jumpBec, bestBecP) < 0)
			{
				isOkBestBecP = true;
				bestBecP = jumpBec;

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
	}

	//не защитник, либо не нашли точку удара для робота с защитной точкой перед воротами
	const auto isForward = !isDefender || movePoint == std::nullopt && &defenderPoint == &_beforeMyGates;

	if (!isForward)
	{		
		if (movePoint == std::nullopt) {
			isOkBestBecP = false;
			targetVelocity = GetDefendPointTargetVelocity(me, defenderPoint);
		}
		else
		{
			isOkBestBecP = true;
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
			isOkBestBecP = false;
			if (&defenderPoint == &_beforeMyGates && //только нап бежит за мячом				
				Helper::GetLength2(Vector3D(_ball.x, me.y, _ball.z), Helper::GetRobotPosition(me)) >
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
		else
		{
			isOkBestBecP = true;
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
	int t, int directionCoeff, std::optional<double>& collisionT, std::optional<BallEntity>& bestBallEntity, double& goalTime)
{
	collisionT = std::nullopt;
	bestBallEntity = std::nullopt;

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

	std::optional<BallEntity> ball_entity = std::nullopt;
	std::optional<double> jumpCollisionT = std::nullopt;

	const bool isOkPosToStrike = directionCoeff == 1 ?
		IsOkPosToJump(robotE, t, jumpCollisionT, ball_entity, goalTime) :
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
		bestBallEntity = ball_entity;
		return true;
	}

	return false;
}

std::optional<Vector3D> MyStrategy::GetAttackerMovePoint(const model::Robot & robot,
	int startAttackTick,
	bool & isDefender, bool& isDefenderSavedPointOk, BallEntityContainer& bestBecP)
{
	isDefenderSavedPointOk = false;
	std::optional<Vector3D> movePoint = std::nullopt;
	isDefender = false;
	const auto tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;

	if (_defenderMovePoints.count(robot.id) > 0)
	{
		const auto tBall = _defenderMovePoints[robot.id].first;
		const auto tJump = _defenderMovePoints[robot.id].second;

		if (tBall > 0 && tJump > 0 && tBall <= BallMoveTicks) //tBall > BallMoveTicks может получиться, если нашли точку для атаки в SetAttAction со StartTick > 1
		{
			bool isPassedBy = false;
			int bestMoveT;
			BallEntityContainer dmpBecP;
			const bool isOk = GetDefenderStrikeBallEntity(robot, tBall, tJump, tJump + 1, dmpBecP, bestMoveT);

			if (isOk && dmpBecP.isGoalScored && (!_oppStrikeTime.has_value() || dmpBecP.collisionTime < _oppStrikeTime.value() - tickTime))
			{
				isDefenderSavedPointOk = true;
				isDefender = true;
				bestBecP = dmpBecP;

				const auto be = _ballEntities.at(tBall);
				for (auto it = _defenderMovePoints.cbegin(); it != _defenderMovePoints.cend();)//удаляем все остальные схраненки
				{
					if ((*it).first != robot.id)
						_defenderMovePoints.erase(it);
					++it;
				}

				return Vector3D(be.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
					be.Position.Z);
			}
		}
		if (tJump != 0)//это прыжок - его обработаем потом
			_defenderMovePoints.erase(robot.id);
	}

	//int bestT = -1;
	bool isResOk = false;
	for (int t = startAttackTick; t <= startAttackTick + BallMoveTicks; ++t)
	{
		if (_goalScoringTick >= 0 && t >= _goalScoringTick)
			return movePoint;
		if (_meGoalScoringTick >= 0 && t >= _meGoalScoringTick)
			return movePoint;
		if (_ballEntities.count(t) == 0) return movePoint;

		auto ballEntity = _ballEntities.at(t);		

		if (IsPenaltyArea(ballEntity.Position, false))//включаем режим защитника
		{
			std::optional<double> curCollisionT = std::nullopt;
			bool isPassedBy = false;
			int bestMoveT;
			BallEntityContainer becP;
			const bool isOk = GetDefenderStrikeBallEntity(robot, t, startAttackTick, t, becP, bestMoveT);
			if (!isOk) continue;

			if (!isResOk || CompareBeContainers(becP, bestBecP) < 0)
			{
				isResOk = true;
				movePoint = Vector3D(ballEntity.Position.X, Constants::Rules.ROBOT_RADIUS, ballEntity.Position.Z);
				isDefender = true;
				bestBecP = becP;
				if (becP.isGoalScored)
				{
					_defenderMovePoints[robot.id] = std::pair<int, int>();
					_defenderMovePoints[robot.id].first = t;
					_defenderMovePoints[robot.id].second = bestMoveT;
				}
				else
				{
					if (_defenderMovePoints.count(robot.id) > 0)
						_defenderMovePoints.erase(robot.id);					
				}
			}			
		}
		else
		{
			std::optional<double> jumpCollisionT = std::nullopt;
			std::optional<BallEntity> curBallEntity = std::nullopt;
			double goalTime;
			Vector3D posToSave;
			const auto attackMovePoint = GetAttackerStrikePoint(robot, t, 1, jumpCollisionT, curBallEntity, goalTime, posToSave);
			
			if (attackMovePoint == std::nullopt)
				continue;

			const auto curCollisionT = t * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();
			BallEntityContainer bec = BallEntityContainer(curBallEntity.value(), curCollisionT, true, goalTime, 0);
			if (!isResOk || CompareBeContainers(bec, bestBecP) < 0)
			{
				_beforeStrikePoints[robot.id] = std::pair<int, Vector3D>(t, posToSave);
				isResOk = true;
				movePoint = attackMovePoint;
				isDefender = false;
				bestBecP = bec;
			}			
		}
	}

	if (movePoint != std::nullopt)
	{
		_drawSpheres.emplace_back((*movePoint).X, (*movePoint).Y, (*movePoint).Z, 0.25, 1, 0, 0, 0.5);
		/*const auto be = _ballEntities.at(bestT);
		_drawSpheres.push_back(Sphere(
			be.Position.X, be.Position.Y, be.Position.Z,
			Constants::Rules.BALL_RADIUS, 1, 0, 0, 0));*/
	}

	return movePoint;
}

std::optional<Vector3D> MyStrategy::GetAttackerStrikePoint(const model::Robot & robot, int t,
	int directionCoeff,
	std::optional<double>& collisionT, std::optional<BallEntity>& bestBallEntity, double& goalTime, Vector3D& posToSave)
{
	const BallEntity ballEntity = _ballEntities.at(t);
	if (_beforeStrikePoints.count(robot.id) > 0)
	{
		if (_beforeStrikePoints[robot.id].first == t)
		{
			if (IsOkPosToMove(_beforeStrikePoints[robot.id].second, robot, t, directionCoeff, collisionT, bestBallEntity, goalTime))
			{
				return _beforeStrikePoints[robot.id].second;
			}			
			_beforeStrikePoints.erase(robot.id);			
		}
	}

	//код, чтобы сразу отбросить лишние варианты. TODO: оптимизировать
	/*if (directionCoeff == 1)
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
	}*/

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
//TODO: здесь тоже можно выбирать лучший goalTime
	if (IsOkPosToMove(mePos, robot, t, directionCoeff, collisionT, bestBallEntity, goalTime))
	{
		return mePos;
	}

	for (int i = 1; i <= StrikeSphereStepsCount; ++i)
	{
		Vector3D rVectorMult1 = rVector1 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos1 = Vector3D(ballEntity.Position.X + rVectorMult1.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult1.Z);
		if (IsOkPosToMove(mePos1, robot, t, directionCoeff, collisionT, bestBallEntity, goalTime))
		{
			return mePos1;
		}

		Vector3D rVectorMult2 = rVector2 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos2 = Vector3D(ballEntity.Position.X + rVectorMult2.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult2.Z);
		if (IsOkPosToMove(mePos2, robot, t, directionCoeff, collisionT, bestBallEntity, goalTime))
		{
			
			return mePos2;
		}
	}

	return std::nullopt;
}

bool MyStrategy::IsOkPosToJump(
	RobotEntity & robotEntity,
	int beforeTicks,
	std::optional<double>& collisionT, std::optional<BallEntity>& bestBallEntity, double& goalTime)
{
	const auto directionCoeff = 1;

	/*if (_ballEntities.at(beforeTicks).Velocity.Z <=0 && 
		Helper::GetLength2(robotEntity.Position, _ballEntities.at(beforeTicks).Position) > _maxStrikeDist2 + EPS)
	{
		collisionT = std::nullopt;
		return false;
	}*/

	//auto const prevBallEntity = BallEntity(ballEntity);

	auto ballEntity = BallEntity(_ballEntities.at(beforeTicks));
	std::vector<BallEntity> resBes;
	std::vector<double> resCollisionTimes;
	auto const isCollision = SimulateCollision(ballEntity, robotEntity, _allHitEs, 3, resBes, resCollisionTimes, beforeTicks, true, false);
	if (!isCollision) return false;

	for (int i = 0; i < 3; ++i)
	{
		auto beCur = resBes.at(i);
		auto const jumpCollisionTCur = resCollisionTimes.at(i);

		if (i == 0)
		{
			bestBallEntity = BallEntity(beCur);
			bestBallEntity->IsCollided = false;
			bestBallEntity->IsArenaCollided = false;
			collisionT = jumpCollisionTCur;
		}

		if (!beCur.IsCollided)
		{
			if (!_isNoCollisionGoalPossible) return false; //не смысла бить по роботу - без роботов мяч не летит в вопрота
			if (_isGoalPossible) return false;//нет смысла бить по роботу - мяч летит в ворота
			// TODO: здесь не выходим. надо проверить, что мяч попадет в ворота. может быть удар по роботу, после его коллизии с мячом
		}
		beCur.IsCollided = false;
		beCur.IsArenaCollided = false;

		//коллизи¤ в штрафной площади
		/*if (IsPenaltyArea(beCur.Position, false))
			return false;*/

		//double angle = GetVectorAngleToHorizontal(beCur.Velocity);
		//if (angle * directionCoeff > M_PI / 3) return false; //TODO: не бьем под большим углом к горизонтали

		double ballFlyTime = 0;
		double changeDirVz = 0;
		if (!IsGoalBallDirection2(beCur, directionCoeff, false, ballFlyTime, changeDirVz)) return false;
		if (i==0)
		{
			goalTime = ballFlyTime;
		}
	}
	return true;
}

bool MyStrategy::IsOkOppPosToJump(
	RobotEntity & robotEntity, int beforeTicks, std::optional<double>& collisionT)
{
	const auto directionCoeff = -1;
	//auto const prevBallEntity = BallEntity(ballEntity);


	auto ballEntity = BallEntity(_ballEntities.at(beforeTicks));
	std::vector<BallEntity> resBes;
	std::vector<double> resCollisionTimes;
	auto const isCollision = SimulateCollision(ballEntity, robotEntity, _averageHitE, 1, resBes, resCollisionTimes, beforeTicks, true, false);
	if (!isCollision) return false;

	double goalTime = -1;
	for (int i = 0; i < 1; ++i)
	{
		auto beCur = resBes.at(i);
		beCur.IsCollided = false;
		beCur.IsArenaCollided = false;
		auto const jumpCollisionTCur = resCollisionTimes.at(i);
		
		collisionT = jumpCollisionTCur;
		double changeDirVz = 0;
		if (!IsGoalBallDirection2(beCur, directionCoeff, false, goalTime, changeDirVz)) return false;
	}	
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

	std::optional<double> collisionT;	

	for (const auto & robot : oppRobots)
	{
		if (!robot.touch) continue;
		auto re = RobotEntity(robot);
		if (IsOkOppPosToJump(re, 0, collisionT))
		{
			return collisionT;
		}
	}

	std::optional<double> minT = std::nullopt;
	double goalTime;

	for( auto t = 1; t <= BallMoveTicks; ++t)
	{
		for (const auto & robot : oppRobots)
		{
			if (!robot.touch) continue;
			std::optional<double> jumpCollisionT = std::nullopt;
			std::optional<BallEntity> bestBallEntity = std::nullopt;
			Vector3D posToSave;
			auto movePoint = GetAttackerStrikePoint(
				robot, t, -1, jumpCollisionT, bestBallEntity, goalTime, posToSave);
			if (movePoint != std::nullopt)
			{
				collisionT = t * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();
				if (!minT.has_value() || collisionT.value() < minT.value())
				{
					_beforeStrikePoints[robot.id] = std::pair<int, Vector3D>(t, posToSave);
					minT = collisionT;
				}
			}
		}
	}
	return minT;
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
	std::map<int, std::optional<double>>& collisionTimes,
	std::map<int, BallEntity>& bestBallEntities)
{
	bool isGoalScored = false;
	_ballEntities = std::map<int, BallEntity>();	
	auto ball_entity = BallEntity(_ball);
	_ballEntities[0] = ball_entity;

	_robotEntities = std::map<int, std::vector<RobotEntity>>();
	_robotEntities[0] = std::vector<RobotEntity>();
	for (auto & robot : _robots)
	{
		if (!robot.touch)
		{
			auto re = RobotEntity(robot);
			if (abs(robot.radius - Constants::Rules.ROBOT_MAX_RADIUS) < EPS)
				re.Action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
			_robotEntities[0].push_back(re);
		}
	}	
	
	bool gotOppCollision = false;
	for (auto t = 1; t <= BallMoveTicks; ++t)
	{
		std::vector<RobotEntity> jumpResCur = std::vector<RobotEntity>();
		for (auto & re: _robotEntities.at(t-1))
		{
			jumpResCur.push_back(RobotEntity(re));
		}

		ball_entity = SimulateTickBall(ball_entity, jumpResCur, isGoalScored, false);
		ball_entity.IsCollided = false;
		ball_entity.IsArenaCollided = false;

		if (!_isGoalPossible && ball_entity.Position.Z > Constants::Rules.arena.depth / 2 + ball_entity.Radius)
		{
			_isGoalPossible = true;
		}

		if (!_isMeGoalPossible && ball_entity.Position.Z < -Constants::Rules.arena.depth / 2 - ball_entity.Radius)
		{
			_isMeGoalPossible = true;
			_meGoalScoringTick = t;
		}


		_ballEntities[t] = ball_entity;

		bool isOppCollided = false;
		for (auto& re : jumpResCur)//TODO: считать только коллизию с мячом, а не друг с другом
		{
			if (re.IsCollided)
			{				
				bool isTeammate = false;
				for (auto & r : _robots)
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
				else
				{
					isOppCollided = true;
				}
				re.IsCollided = false;
			}
		}

		if (!gotOppCollision)
		{
			if (isOppCollided)
				gotOppCollision = true;
			else
			{
				if (!_isNoCollisionGoalPossible && ball_entity.Position.Z > Constants::Rules.arena.depth / 2 + ball_entity.Radius)
				{
					_isNoCollisionGoalPossible = true;
				}

				if (!_isNoCollisionMeGoalPossible && ball_entity.Position.Z < -Constants::Rules.arena.depth / 2 - ball_entity.Radius)
				{
					_isNoCollisionMeGoalPossible = true;
				}
			}
		}

		jumpResCur.erase(
			std::remove_if(
				jumpResCur.begin(), jumpResCur.end(), [](RobotEntity re) {return re.IsArenaCollided; }), jumpResCur.end());

		_robotEntities[t] = jumpResCur;		

	}


	//если была коллизия с роботом, считаем еще раз без роботов
	if (gotOppCollision)
	{
		ball_entity = BallEntity(_ball);
		std::vector<RobotEntity> noRes = std::vector<RobotEntity>();
		for (auto t = 1; t <= BallMoveTicks; ++t)
		{
			ball_entity = SimulateTickBall(ball_entity, noRes, isGoalScored, true);

			if (!_isNoCollisionGoalPossible && ball_entity.Position.Z > Constants::Rules.arena.depth / 2 + ball_entity.Radius)
			{
				_isNoCollisionGoalPossible = true;
			}

			if (!_isNoCollisionMeGoalPossible && ball_entity.Position.Z < -Constants::Rules.arena.depth / 2 - ball_entity.Radius)
			{
				_isNoCollisionMeGoalPossible = true;
			}
		}
	}

	
	for (auto t = 1; t <= BallMoveTicks; ++t)
	{
		const auto be = _ballEntities.at(t);
		_drawSpheres.push_back(Sphere(
			be.Position.X,
			be.Position.Y,
			be.Position.Z,
			2, 1, 1, 0, 0.25
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

	if (_ballEntities.count(preCollisionTick) == 0) return afterCollisionTick;

	auto ballEntity = BallEntity(_ballEntities.at(preCollisionTick));
	auto robotEntities = std::vector<RobotEntity>();
	for (const auto & re : _robotEntities.at(preCollisionTick))
		robotEntities.push_back(RobotEntity(re));

	const auto beforeCollisionDeltaTime = collisionTime - preCollisionTick * 1.0 / Constants::Rules.TICKS_PER_SECOND;
	bool isGoalScored;
	double hitE = (_hitEs[0] + _hitEs[1]) / 2.0;

	Simulator::UpdateOnAir(ballEntity, robotEntities, beforeCollisionDeltaTime, hitE, isGoalScored, false);

	const auto afterCollisionDeltaTime = afterCollisionTick * 1.0 / Constants::Rules.TICKS_PER_SECOND - collisionTime;
	ballEntity.Velocity = Vector3D(afterCollisionBallVelocity);
	Simulator::UpdateOnAir(ballEntity, robotEntities, afterCollisionDeltaTime, hitE, isGoalScored, false);

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

		//TODO: спорная штука
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