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
	_robotsCounter = (_robotsCounter + 1) % (game.robots.size()/2);

	if (abs(game.ball.z) > rules.arena.depth / 2 + game.ball.radius) //goal scored
	{
		return;
	}

	if (_robotsCounter == 0)
	{		
		_lastMyCollisionTick = -1;
		_oppBallCollisionTicks = std::map<int, int>();
		_minOppCollisionTick = -1;
		_meGoalScoringTick = -1;
		_goalScoringTick = -1;
		_noCollisionGoalScoringTick = -1;
		_isGoalPossible = false;
		_isMeGoalPossible = false;
		_isNoCollisionGoalPossible = false;
		_isNoCollisionMeGoalPossible = false;

		_ball = game.ball;
		_robots = game.robots;
		_oppStrikeTime = std::nullopt;
		_drawSpheres = std::vector<Sphere>();
		InitBallEntities();
		_actions = std::map<int, model::Action > ();

		//_nitroPosCur = std::map<int, Vector3D>();
		_nitroTicksCur = std::map<int, int>();

		std::vector<Robot> opp_robots = std::vector<Robot>();
		for (Robot robot : game.robots)
		{
			if (!robot.is_teammate) opp_robots.push_back(robot);
		}
		if (!_oppStrikeTime.has_value())//�.�. ����������� � InitBallEntities
			_oppStrikeTime = GetOppStrikeTime(opp_robots);
		
		
	}
	else
	{
		InitAction(action, me.id);
		return;
	}
	

	
	auto const ballEntity = BallEntity(game.ball);

	std::vector <Robot> myRobots = std::vector<Robot>();
	Robot defender{};
	Robot attacker{};
	bool isJumping = false;

	for (Robot robot : game.robots)
	{
		if (!robot.is_teammate) continue;
		myRobots.push_back(robot);
		if (!robot.touch)
		{
			isJumping = true;
			InitJumpingRobotAction(robot, game.ball);
		}

		bool meIsDefender = true;
		bool meIsAttacker = true;
		for (Robot r : game.robots)
		{
			if (!r.is_teammate) continue;
			if (r.id == robot.id) continue;
			if (r.z < robot.z) meIsDefender = false;
			if (r.z > robot.z) meIsAttacker = false;
		}
		if (meIsDefender)
			defender = robot;
		if (meIsAttacker)
			attacker = robot;
	}


	int gkAttId = -1;
	if (_isGoalPossible)
	{
		Robot gk{};
		double maxZ = -50;
		for (auto &r : _robots)
		{
			if (r.is_teammate) continue;
			if (!r.touch) continue;
			if (r.z > maxZ)
			{
				maxZ = r.z;
				gk = r;
			}
		}

		if (abs(maxZ + 50) > EPS &&
			(attacker.x - gk.x)*(attacker.x - gk.x) + (attacker.y - gk.y) * (attacker.y - gk.y) + (attacker.z - gk.z)*(attacker.z - gk.z) <=
			(4 * Constants::Rules.ROBOT_MIN_RADIUS) * (4 * Constants::Rules.ROBOT_MIN_RADIUS))
		{
			const auto attPos = Helper::GetRobotPosition(attacker);
			auto oppPos = Helper::GetRobotPosition(gk);
			const double tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
			oppPos.X += gk.velocity_x * tickTime;
			oppPos.Z += gk.velocity_z * tickTime;
			const auto targetVelocity = Helper::GetTargetVelocity(attPos, oppPos, Constants::Rules.ROBOT_MAX_GROUND_SPEED);

			model::Action gkAction = model::Action();
			gkAction.target_velocity_x = targetVelocity.X;
			gkAction.target_velocity_y = 0.0;
			gkAction.target_velocity_z = targetVelocity.Z;
			gkAction.jump_speed = 0.0;
			gkAction.use_nitro = false;
			_actions[attacker.id] = gkAction;
			gkAttId = attacker.id;
		}
	}


	const int startAttackTick = _lastMyCollisionTick == -1 || _isMeGoalPossible ? 0 : _lastMyCollisionTick + AttackerAddTicks;
	int bestBecPRobotId = -1;
	BallEntityContainer bestBec;

	for (auto & robot : myRobots)
	{
		if (!robot.touch) continue;
		if (robot.id == gkAttId) continue;
		BallEntityContainer curBestBec;
		bool isOkBestBec;
		int position = robot.id == defender.id ? -1 : robot.id == attacker.id ? 1 : 0;

		const Action robotAction = SetAttackerAction(
			robot,
			startAttackTick, //��������� ��� �����		
			curBestBec, isOkBestBec, position);
		if (isOkBestBec)
		{			
			if (robot.id == defender.id)
				if (_oppStrikeTime.has_value() && curBestBec.collisionTime > _oppStrikeTime.value() &&
					curBestBec.ResBallEntity.Position.Z > -Constants::Rules.arena.depth / 4)
					continue;//������� ���������, ������� ����������� ����� ������� �� �����

			if (bestBecPRobotId == -1 || CompareBeContainers(curBestBec, bestBec) < 0)
			{
				bestBec = curBestBec;
				bestBecPRobotId = robot.id;
			}
		}
		_actions[robot.id] = robotAction;
	}

	for (auto& robot:myRobots)
	{
		if (robot.id == bestBecPRobotId)
			continue;
		if (_defenderMovePoints.count(robot.id) > 0)
			_defenderMovePoints.erase(robot.id);
	}

	std::set<int> goNitroRobots = std::set<int>();

	if (bestBecPRobotId != -1)//����� �������
	{
		const auto collisionTime = bestBec.collisionTime;
		const auto afterCollisionTick = UpdateBallEntities(collisionTime, bestBec.ResBallEntity.Velocity, bestBec.isGoalScored);

		if (!_oppStrikeTime.has_value() || bestBec.collisionTime < _oppStrikeTime.value())//� �� �������� �����
		{			
			if (bestBecPRobotId != defender.id) //���� �� ��������
			{				
			
				for (auto& robot : myRobots) //����� ��������� ���������
				{
					if (!robot.touch) continue;
					if (robot.id == gkAttId) continue;
					if (robot.id == bestBecPRobotId)
					{
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 1, 0, 0.5);
						continue;
					}
					if (robot.id == defender.id)
					{
						if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75 && IsSafoToCollectNitro())
							goNitroRobots.insert(robot.id);

						int runTicks = -1;
						_actions[robot.id] = GetDefaultAction(robot, _myGates, runTicks);//��� ���� �� ������
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0, 0, 1, 0.5);
						
						continue;
					}

					if (_goalScoringTick != -1) //���� ���������� �������
					{
						int resIndex;
						_actions[robot.id] = GetMoveKeeperOrOppAction(robot, resIndex);
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1,
							resIndex == 1 ? 1 : 0, resIndex == 1 ? 0.5 : 0, resIndex == 1 ? 0.5 : 0, 0.5);
					}
					else
					{
						BallEntityContainer curBestBec;
						bool isOkBestBec;
						int position = robot.id == defender.id ? -1 : robot.id == attacker.id ? 1 : 0;
						const Action attAction = SetAttackerAction(
							robot, afterCollisionTick + AttackerAddTicks, curBestBec, isOkBestBec, position);

						if (isOkBestBec)//����� ����� ����� - ���� � ���
						{
							_actions[robot.id] = attAction;
							_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0.5, 1, 0.5, 0.5);
						}
						else //���� ���������� ����������
						{
							if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75)
								goNitroRobots.insert(robot.id);
							_actions[robot.id] = GetNearestOppAttackAction(robot, -1);
							_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);

						}
					}

					
				}
			}
			else//���� ��������
			{
				int nextBestBecPRobotId = -1;
				BallEntityContainer nextBestBec;
				for (auto& robot : myRobots) //���������� ������� �����������
				{
					if (!robot.touch) continue;
					if (robot.id == gkAttId) continue;
					if (robot.id == bestBecPRobotId) 
					{
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 1, 0, 0.5);
						continue;
					}

					BallEntityContainer curBestBec;
					bool isOkBestBec;
					int position = robot.id == defender.id ? -1 : robot.id == attacker.id ? 1 : 0;
					const Action attAction = SetAttackerAction(
						robot, afterCollisionTick + AttackerAddTicks, curBestBec, isOkBestBec, position);

					if (isOkBestBec)
					{
						if (nextBestBecPRobotId == -1 || CompareBeContainers(curBestBec, nextBestBec) < 0)
						{
							nextBestBec = curBestBec;
							nextBestBecPRobotId = robot.id;
						}
					}
					_actions[robot.id] = attAction;
				}

				if (nextBestBecPRobotId != -1)//����� �����������
				{
					for (auto& robot : myRobots) //��������� ������� ����������
					{
						if (!robot.touch) continue;
						if (robot.id == gkAttId) continue;
						if (robot.id == bestBecPRobotId) continue;
						if (robot.id == nextBestBecPRobotId)
						{
							_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0.5, 1, 0.5, 0.5);
							continue;
						}
						if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75)
							goNitroRobots.insert(robot.id);
						_actions[robot.id] = GetNearestOppAttackAction(robot, -1);
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
					}
				}
				else//�� ����� �����������
				{
					for (auto& robot : myRobots)
					{
						if (!robot.touch) continue;
						if (robot.id == bestBecPRobotId) continue;
						if (robot.id == gkAttId) continue;
						if (robot.id == attacker.id)
						{

							if (_isGoalPossible) //���� ���������� �������
							{
								int resIndex;
								_actions[robot.id] = GetMoveKeeperOrOppAction(robot, resIndex);
								_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1,
									resIndex == 1 ? 1 : 0, resIndex == 1 ? 0.5 : 0, resIndex == 1 ? 0.5 : 0, 0.5);
							}
							else//������� �����
							{
								if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75)
									goNitroRobots.insert(robot.id);
								const auto nearestOpp = get_my_gates_opp_robot(-1);
								_actions[robot.id] = GetNearestOppAttackAction(robot, nearestOpp.id);
								_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
							}
						}
						else//�� ������� �����
						{
							if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75)
								goNitroRobots.insert(robot.id);
							_actions[robot.id] = GetNearestOppAttackAction(robot, -1);
							_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
						}						
					}
				}
			}
		}
		else//��� ������ �� �������� �����
		{
			if (bestBecPRobotId == defender.id)//���� ��������
			{
				for (auto& robot : myRobots)
				{
					if (!robot.touch) continue;
					if (robot.id == gkAttId) continue;
					if (robot.id == bestBecPRobotId) 
					{
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 1, 0, 0.5);
						continue;
					}
					if (robot.id == attacker.id)//��� ������� �����
					{
						_actions[robot.id] = GetNearestOppAttackAction(robot, -1);
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
					}
					else//�� ���� �� ������
					{
						int runTicks = -1;
						_actions[robot.id] = GetDefaultAction(robot, _myGates, runTicks);
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0, 0, 1, 0.5);
					}
				}
			}
			else //���� ��� ��� ��
			{				
				for (auto& robot : myRobots) //��������� ���������
				{
					if (!robot.touch) continue;
					if (robot.id == gkAttId) continue;
					if (robot.id == bestBecPRobotId)
					{
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 1, 0, 0.5);
						continue;
					}
					if (robot.id == defender.id)
					{
						int runTicks = -1;
						if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75 && IsSafoToCollectNitro())
							goNitroRobots.insert(robot.id);
						_actions[robot.id] = GetDefaultAction(robot, _myGates, runTicks);//���. ���� �� ������
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0, 0, 1, 0.5);
						continue;
					}

					if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75)
						goNitroRobots.insert(robot.id);
					
					BallEntityContainer curBestBec;
					bool isOkBestBec;
					int position = robot.id == defender.id ? -1 : robot.id == attacker.id ? 1 : 0;
					const Action attAction = SetAttackerAction(
						robot, afterCollisionTick + AttackerAddTicks, curBestBec, isOkBestBec, position);
					if (isOkBestBec)//���� � ����� ���������
					{
						_actions[robot.id] = attAction;
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0.5, 1, 0.5, 0.5);
					}
					else//������� �����
					{
						_actions[robot.id] = GetNearestOppAttackAction(robot, -1);
						_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
					}
					
				}
			}
		}
	}
	else //�� ����� ������ ������
	{	
		int saveGatesId = -1;
		if (_meGoalScoringTick != -1 && _meGoalScoringTick <= 50)//������� ������, ���� ����
		{
			if (defender.touch)
			{
				bool canBeSaved;
				const auto saveGatesAction = GetSaveGatesAction(defender, canBeSaved);
				if (canBeSaved)
				{
					_actions[defender.id] = saveGatesAction;
					saveGatesId = defender.id;
				}
			}

			if (saveGatesId == -1)
			{
				for (auto & robot : myRobots)
				{
					if (!robot.touch) continue;
					if (robot.id == defender.id || robot.id == attacker.id) continue;
					if (robot.z > 0) continue;

					bool canBeSaved;
					const auto saveGatesAction = GetSaveGatesAction(robot, canBeSaved);
					if (canBeSaved)
					{
						_actions[robot.id] = saveGatesAction;
						saveGatesId = robot.id;
					}
				}

				if (saveGatesId == -1)
				{
					if (attacker.touch && attacker.z > 0)
					{
						bool canBeSaved;
						const auto saveGatesAction = GetSaveGatesAction(attacker, canBeSaved);
						if (canBeSaved)
						{
							_actions[attacker.id] = saveGatesAction;
							saveGatesId = attacker.id;
						}
					}
				}
			}
		}

		for (auto& robot : myRobots)
		{
			if (!robot.touch) continue;
			if (robot.id == saveGatesId) continue;
			if (robot.id == gkAttId) continue;
			if (robot.id == defender.id)
			{
				if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75 && IsSafoToCollectNitro())
				{
					goNitroRobots.insert(robot.id);
				}

				int runTicks = -1;
				_actions[robot.id] = GetDefaultAction(defender, _myGates, runTicks);//��� ���� �� ������
				_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0, 0, 1, 0.5);
			}
			else if (robot.id == attacker.id)//��� ���� �� ����� ��� �� ����������
			{
				if (_isGoalPossible) //���� ���������� �������
				{
					int resIndex;
					_actions[robot.id] = GetMoveKeeperOrOppAction(robot, resIndex);
					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1,
						resIndex == 1 ? 1 : 0, resIndex == 1 ? 0.5 : 0, resIndex == 1 ? 0.5 : 0, 0.5);
				}
				else//������� �����
				{
					if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75)
						goNitroRobots.insert(robot.id);

					int excludeId = -1;
					if (!(_isMeGoalPossible && !defender.touch))
					{
						const auto nearestOpp = get_my_gates_opp_robot(excludeId);
						excludeId = nearestOpp.id;
					}

					_actions[robot.id] = GetNearestOppAttackAction(robot, excludeId);
					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
				}				
				
			}
			else//�� ������� ����� ��� ���� �� ������, ���� ��� ��� ��������� � �������� ���
			{
				if (_isMeGoalPossible && !defender.touch)
				{
					int runTicks = -1;
					_actions[robot.id] = GetDefaultAction(robot, _myGates, runTicks);
					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0, 0, 1, 0.5);
				}
				else
				{
					if (robot.nitro_amount <= Constants::Rules.MAX_NITRO_AMOUNT * 0.75)
						goNitroRobots.insert(robot.id);
					_actions[robot.id] = GetNearestOppAttackAction(robot, -1);
					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
				}
				
			}
		}
	}


	std::set<int> okRobots = std::set<int>();
	std::set<int> okNitroPacks = std::set<int>();	

	while (true)
	{
		std::map<int, std::pair<int, double>> bestNps = std::map<int, std::pair<int, double>>();
		for (auto gnRobotId : goNitroRobots)
		{
			if (okRobots.find(gnRobotId) != okRobots.end())
				continue;

			Robot gnRobot = GetRobotById(gnRobotId);			

			auto minDist2 = std::numeric_limits<double>::max();
			std::optional<NitroPack> res = std::nullopt;
			for (const auto& np : game.nitro_packs)
			{
				if (!np.alive)
					continue;

				if (okNitroPacks.find(np.id) != okNitroPacks.end())
					continue;

				if (np.z * gnRobot.z < 0)
					continue; //���� ������ �� ����� ��������

				const auto dist2 = (np.x - gnRobot.x)*(np.x - gnRobot.x) + (np.z - gnRobot.z)*(np.z - gnRobot.z);
				if (dist2 < minDist2)
				{
					minDist2 = dist2;
					res = np;
				}
			}

			if (!res.has_value())//�� ����� ����� ��� ����� ������
			{
				okRobots.insert(gnRobot.id);
				continue;
			}

			bestNps[gnRobot.id] = std::make_pair(res.value().id, minDist2);

		}

		bool isGot = false;
		int bestRobotId = -1;
		for (auto & bestNp: bestNps)
		{
			if (!isGot || bestNp.second.second < bestNps.at(bestRobotId).second)
			{
				bestRobotId = bestNp.first;
				isGot = true;
			}
		}

		if (isGot)
		{
			okRobots.insert(bestRobotId);
			const auto nitroId = bestNps.at(bestRobotId).first;
			okNitroPacks.insert(nitroId);

			int runTicks = -1;
			const auto bestRobot = GetRobotById(bestRobotId);
			const auto nitroPack = GetNitroById(game, nitroId);
			const auto npPos = Vector3D(nitroPack.x, Constants::Rules.ROBOT_MIN_RADIUS, nitroPack.z);
			_actions[bestRobotId] = GetDefaultAction(bestRobot, npPos, runTicks);
			_drawSpheres.emplace_back(bestRobot.x, bestRobot.y, bestRobot.z, 1, 1, 0, 1, 0.5);
		}

		if (okRobots.size() == goNitroRobots.size())
			break;
	}


	/*const auto nearestNitro = get_nearest_nitro_pack(robot, game);
	if (nearestNitro.has_value())
	{
		const auto nnValue = nearestNitro.value();
		const auto nnPos = Vector3D(nnValue.x, nnValue.y, nnValue.z);
		int runTicks = -1;
		_actions[robot.id] = GetDefaultAction(robot, nnPos, runTicks);
		_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0, 1, 0.5);
	}*/
	



	for (auto& act:_actions)
	{
		//if (act.second.use_nitro && _nitroPosCur.count(act.first) > 0)
		//{
		//	_usingNitroIds.insert(act.first);
		//	//_nitroPositions[act.first] = _nitroPosCur[act.first];
		//}
		if (abs(act.second.jump_speed-Constants::Rules.ROBOT_MAX_JUMP_SPEED) <EPS  &&
			_nitroTicksCur.count(act.first) > 0)
		{
			//_usingNitroIds.insert(act.first);
			_nitroTicks[act.first] = _nitroTicksCur[act.first];
		}
	}

	/*const auto nearestNitro = get_nearest_nitro_pack(robot, game);
	if (nearestNitro.has_value())
	{
		const auto nnValue = nearestNitro.value();
		const auto nnPos = Vector3D(nnValue.x, nnValue.y, nnValue.z);
		int runTicks = -1;
		_actions[robot.id] = GetDefaultAction(robot, nnPos, runTicks);
		_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0, 1, 0.5);
	}*/
	



	////bool isDefender = false;
	////bool isStrikeDefender = false;
	//if (isJumping)
	//{
	//	//if (_isMeGoalPossible)
	//	//{
	//	//	//�������� ������, ������� � 1 ����
	//	//	for (auto & robot : myRobots)
	//	//	{
	//	//		if (!robot.touch) continue;
	//	//		std::optional<double> collisionT = std::nullopt;
	//	//		auto best_ball_entity = BallEntity(_ball);
	//	//		Action robotAction = SetDefenderAction(
	//	//			robot, robot.id==defender.id ? _myGates : _beforeMyGates, collisionT, best_ball_entity);
	//	//		collisionTimes[robot.id] = collisionT;
	//	//		bestBallEntities[robot.id] = best_ball_entity;
	//	//		_actions[robot.id] = robotAction;
	//	//	}
	//	//}
	//	//else
	//	//{
	//	//	//���� � �����, ������� � ���������� ���� �������� ����������
	//	//	for (auto & robot : myRobots)
	//	//	{
	//	//		if (!robot.touch) continue;
	//	//		std::optional<double> collisionT = std::nullopt;
	//	//		auto bestBallEntity = BallEntity(_ball);
	//	//		Action robotAction = SetAttackerAction(
	//	//			robot, maxCollisionTick == -1 ? 1 : maxCollisionTick + AttackerAddTicks,
	//	//			robot.id == defender.id ? _myGates : _beforeMyGates, collisionT, bestBallEntity, isDefender, isStrikeDefender);
	//	//		collisionTimes[robot.id] = collisionT;
	//	//		bestBallEntities[robot.id] = bestBallEntity;
	//	//		_actions[robot.id] = robotAction;
	//	//	}
	//	//}

	//	//���� � �����, ������� � ���������� ���� �������� ����������


	//	int nextBestBecPRobotId = -1;
	//	BallEntityContainer nextBestBec;

	//	for (auto & robot : myRobots)
	//	{
	//		if (!robot.touch) continue;			
	//		BallEntityContainer curBestBec;
	//		bool isOkBestBec;
	//		int position = robot.id == defender.id ? -1 : robot.id == attacker.id ? 1 : 0;

	//		const Action robotAction = SetAttackerAction(
	//			robot, 
	//			_lastMyCollisionTick == -1 || _isMeGoalPossible ? 0 : _lastMyCollisionTick + AttackerAddTicks, //��������� ��� �����
	//			robot.id == defender.id ? _myGates : _beforeMyGates, //�����, �� ������� ����� ����������
	//			curBestBec, isOkBestBec, position);
	//		if (isOkBestBec)
	//		{
	//			if (robot.id == defender.id)
	//				if (_oppStrikeTime.has_value() && curBestBec.collisionTime > _oppStrikeTime.value()
	//					&& curBestBec.ResBallEntity.Position.Z > -Constants::Rules.arena.depth / 4)
	//				{
	//					_actions[defender.id] = GetDefaultAction(defender, _myGates);
	//					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0, 1, 0, 0.5);
	//					continue; //�� ������� ���������� �� ����������� �����				
	//				}

	//			if (nextBestBecPRobotId == -1 || CompareBeContainers(curBestBec, nextBestBec) < 0)
	//			{					
	//				nextBestBec = curBestBec;
	//				nextBestBecPRobotId = robot.id;
	//			}				
	//		}

	//		
	//		_actions[robot.id] = robotAction;
	//	}

	//	if (nextBestBecPRobotId != -1)
	//	{
	//		for (auto & robot : myRobots)
	//		{
	//			if (!robot.touch) continue;
	//			if (robot.id == nextBestBecPRobotId)
	//			{
	//				_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0.5, 1, 0.5, 0.5);
	//				continue;
	//			}
	//			if (robot.id == defender.id)
	//			{
	//				_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0, 0, 1, 0.5);
	//				_actions[robot.id] = GetDefaultAction(robot, _myGates);
	//			}
	//			else
	//			{
	//				_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
	//				_actions[robot.id] = GetNearestOppAttackAction(robot);
	//			}
	//		}
	//	}
	//	
	//}
	//else
	//{		
	//	int bestBecPRobotId = -1;
	//	bool isLoosingDefender = false;
	//	BallEntityContainer bestBec;
	//	for (auto& robot:myRobots)
	//	{
	//		if (!robot.touch) continue;
	//		BallEntityContainer curBestBec;
	//		bool isOkBestBec;
	//		int position = robot.id == defender.id ? -1 : robot.id == attacker.id ? 1 : 0;
	//		const auto attAction = SetAttackerAction(robot, 0, robot.id == defender.id ? _myGates : _beforeMyGates, curBestBec, isOkBestBec, position);

	//		if (isOkBestBec)
	//		{
	//			if (robot.id == defender.id)
	//				if (_oppStrikeTime.has_value() && curBestBec.collisionTime > _oppStrikeTime.value() &&
	//					curBestBec.ResBallEntity.Position.Z > -Constants::Rules.arena.depth / 4)
	//				{
	//					isLoosingDefender = true;
	//					continue; //�� ������� ���������� �� ����������� �����				
	//				}

	//			if (bestBecPRobotId == -1 || CompareBeContainers(curBestBec, bestBec) < 0)
	//			{
	//				bestBec = curBestBec;
	//				bestBecPRobotId = robot.id;
	//			}
	//		}
	//		_actions[robot.id] = attAction;			
	//	}

	//	for (auto& robot:myRobots)
	//	{
	//		if (robot.id == bestBecPRobotId)
	//			continue;
	//		if (_defenderMovePoints.count(robot.id) > 0)
	//			_defenderMovePoints.erase(robot.id);
	//	}

	//	if (bestBecPRobotId != -1)
	//	{
	//		const auto isPenaltyAreaStrike = bestBec.ResBallEntity.Position.Z < 0 && (!_oppStrikeTime.has_value() || bestBec.collisionTime < _oppStrikeTime.value());
	//		auto afterCollisionTick = -1;
	//		if (isPenaltyAreaStrike)
	//		{
	//			const auto collisionTime = bestBec.collisionTime;
	//			afterCollisionTick = UpdateBallEntities(collisionTime, bestBec.ResBallEntity.Velocity, bestBec.isGoalScored);
	//		}

	//		int nextBestBecPRobotId = -1;
	//		BallEntityContainer nextBestBec;

	//		for (auto & robot : myRobots)
	//		{
	//			if (!robot.touch) continue;
	//			if (robot.id == bestBecPRobotId)
	//			{
	//				_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 1, 0, 0.5);
	//				continue;
	//			}

	//			/*if (_defenderMovePoints.count(robot.id) > 0)
	//				_defenderMovePoints.erase(robot.id);*/

	//			if (isPenaltyAreaStrike && (game.robots.size() == 4 || game.robots.size() == 6 && robot.id != defender.id))//���� �� ����� �������� - ������ ���� ��������
	//			{
	//				BallEntityContainer curBestBec;
	//				bool isOkBestBec;
	//				int position = robot.id == defender.id ? -1 : robot.id == attacker.id ? 1 : 0;
	//				const Action attAction = SetAttackerAction(
	//					robot, afterCollisionTick + AttackerAddTicks, _beforeMyGates, curBestBec, isOkBestBec, position);

	//				if (isOkBestBec)
	//				{
	//					if (nextBestBecPRobotId == -1 || CompareBeContainers(curBestBec, nextBestBec) < 0)
	//					{
	//						nextBestBec = curBestBec;
	//						nextBestBecPRobotId = robot.id;
	//					}
	//				}					
	//				_actions[robot.id] = attAction;
	//			}
	//			else //���� � ����� �������� - ������ ���� �� ������
	//			{
	//				if (robot.id == defender.id)
	//				{
	//					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0, 0, 1, 0.5);
	//					_actions[robot.id] = GetDefaultAction(robot, _myGates);
	//				}
	//				else
	//				{
	//					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
	//					_actions[robot.id] = GetNearestOppAttackAction(robot);
	//				}
	//			}
	//		}

	//		if (nextBestBecPRobotId != -1)
	//		{
	//			for (auto & robot : myRobots)
	//			{
	//				if (!robot.touch) continue;
	//				if (robot.id == bestBecPRobotId) continue;
	//				if (robot.id == nextBestBecPRobotId)
	//				{
	//					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0.5, 1, 0.5, 0.5);
	//					continue;
	//				}
	//				if (robot.id == defender.id)
	//				{
	//					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 0, 0, 1, 0.5);
	//					_actions[robot.id] = GetDefaultAction(robot, _myGates);
	//				}
	//				else
	//				{
	//					_drawSpheres.emplace_back(robot.x, robot.y, robot.z, 1, 1, 0.5, 0.5, 0.5);
	//					_actions[robot.id] = GetNearestOppAttackAction(robot);
	//				}
	//			}
	//		}
	//	}
	//	else if (isLoosingDefender)
	//	{
	//		_drawSpheres.emplace_back(defender.x, defender.y, defender.z, 1, 0, 0, 1, 0.5);
	//		const Action defAction = GetDefaultAction(defender, _myGates);
	//		_actions[defender.id] = defAction;
	//	}




	//	int strikeDefenderId = -1;
	//	int bestAttCollisionTId = -1;
	//	
	//	auto gotStriker = false;
	//	for (auto & attacker : attackers) //���� �����, �� ������� ��� ����� ������� �� ������� ����������
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
	//	if (isStrikeDefender)//��� ���� �� ������� �� ���� ������. ��������� ����� ���� ������
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

	//	else if (gotStriker)//����� ����� ��� ����� �����. �������� ���� �� ������
	//	{
	//		Action robotAction = GetDefaultAction(defender, _myGates);
	//		collisionTimes[defender.id] = std::nullopt;
	//		bestBallEntities[defender.id] = BallEntity(game.ball);
	//		_actions[defender.id] = robotAction;
	//	}
	//	else //�� ����� ����� ��� ����� �����.
	//	{
	//		auto bestBallentity = BallEntity(game.ball);
	//		std::optional<double> collisionT = std::nullopt;
	//		Action robotAction = SetDefenderAction(
	//			defender, _myGates, collisionT, bestBallentity);
	//		collisionTimes[defender.id] = collisionT;
	//		bestBallEntities[defender.id] = bestBallentity;
	//		_actions[defender.id] = robotAction;

	//		//����� ����� ��� ���������, � ������� �� �������� ���� �����
	//		if (collisionT != std::nullopt && (_oppStrikeTime == std::nullopt || collisionT.value() < _oppStrikeTime.value()))
	//		{
	//			const auto afterCollisionTick = UpdateBallEntities(
	//				collisionTimes[defender.id].value(), bestBallEntities[defender.id].Velocity);
	//			//���� ���� ������������� �� ����������� ����������
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
	//		else//�������� �� �������� �����
	//		{					
	//			//����� ����� ��� ����, � ������� �� �������� ���� �����
	//			if (bestAttCollisionTId >= 0 &&
	//				(_oppStrikeTime == std::nullopt || collisionTimes[bestAttCollisionTId].value() < _oppStrikeTime.value()))
	//			{
	//				//�������� ���� � �����
	//				//TODO: �� ��������� ���� � ���. ������

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
	//			else //��� �� �������� �����
	//			{
	//				//����� �������� ��� ��� ����� ������. �� ����, ��� ���� �������
	//				//�������� ����������� Action. Action ������� ������ ���������� �� ���������
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
	//							if (!otherRobot.touch ||  //���� ���� ��� � �������, ����������� �� ����� ������
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
	//}		
	
	InitAction(action, me.id);
	for (auto & bsp:_beforeStrikePoints)
	{
		_beforeStrikePoints[bsp.first].first--;
	}

	std::vector<int> removeIds = std::vector<int>();
	for (auto & dmp : _defenderMovePoints)
	{
		auto times = _defenderMovePoints[dmp.first];
		
		auto ballT = std::get<0>(times);
		auto waitT = std::get<1>(times);
		auto moveT = std::get<2>(times);

		std::get<0>(times) = ballT - 1;
		if (waitT > 0)
			std::get<1>(times) = waitT - 1;
		else		
			std::get<2>(times) = moveT - 1;
		
		if (std::get<0>(times) <  0 || std::get<1>(times) < 0 || std::get<2>(times) < 0)
		{
			removeIds.push_back(dmp.first);
		}
		else
		{
			_defenderMovePoints[dmp.first] = times;
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

	if (_nitroTicks.count(robot.id) > 0)
	{
		if (_nitroTicks.at(robot.id) > 0)
		{
			if (robot.z > 0 && !_isGoalPossible)//������ ��������� �����
			{
				auto jumpRes = std::vector<RobotEntity>();
				auto nitroRe = RobotEntity(robot);
				nitroRe.Action.use_nitro = false;
				nitroRe.Action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
				jumpRes.push_back(nitroRe);

				for (const auto & jre : _robotEntities.at(0))
				{
					if (jre.Id == robot.id) continue;
					jumpRes.push_back(jre);
				}
				
				auto be = BallEntity(_ballEntities[0]);
				double afterJumpCollisionT = 0;
				auto const isCol = SimulateFullCollision(be, jumpRes, afterJumpCollisionT,
					_averageHitE[0], false, false);
				if (isCol)
				{
					double gt;
					BallEntity collideBe;
					int collisionsCount;
					bool isGoal = IsGoalBallDirection2(be, 1, true, gt, collideBe, false, collisionsCount);
					if (isGoal)
					{
						_nitroTicks.erase(robot.id);
						robotAction.target_velocity_x = 0;
						robotAction.target_velocity_y = 0;
						robotAction.target_velocity_z = 0;
						robotAction.use_nitro = false;
						robotAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
						_actions[robot.id] = robotAction;
						return;
					}
				}
			}

			robotAction.target_velocity_x = robot.velocity_x;
			robotAction.target_velocity_y = Constants::Rules.MAX_ENTITY_SPEED;
			robotAction.target_velocity_z = robot.velocity_z;
			robotAction.use_nitro = true;
			_nitroTicks[robot.id]--;
		}
		else
		{
			_nitroTicks.erase(robot.id);
		}
	}
	else if (!_isGoalPossible && _ball.z > 0)
	{
		//��������, ����� ������� �� �������

		auto nitroRe = RobotEntity(robot);
		Action nitroAction = Action();
		nitroAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
		nitroAction.target_velocity_x = robot.velocity_x;
		nitroAction.target_velocity_y = Constants::Rules.MAX_ENTITY_SPEED;
		nitroAction.target_velocity_z = robot.velocity_z;
		nitroAction.use_nitro = true;
		nitroRe.Action = nitroAction;

		std::vector<BallEntity> resBes;
		double collisionTime;
		bool is_nitro_collision = simulate_ball_nitro_jump(nitroRe, 0, resBes, collisionTime, false, -1);
		int collisionTicks = int(collisionTime * Constants::Rules.TICKS_PER_SECOND) + 1;
		double goalTime;
		BallEntity collideBe;
		int collisionsCount;
		if (is_nitro_collision && 
			(_minOppCollisionTick == -1 || _minOppCollisionTick != -1 && collisionTicks < _minOppCollisionTick) &&
			IsGoalBallDirection2(resBes[0], 1, true, goalTime, collideBe, false, collisionsCount))
		{
			_actions[robot.id] = nitroAction;
			_nitroTicks[robot.id] = collisionTicks;
			return;
		}

		if (!_isNoCollisionGoalPossible)
		{
			//��������, ����� ������� �� ����������

			for (auto & oppCt : _oppBallCollisionTicks)
			{
				const int oppId = oppCt.first;
				const int oppCollisionTick = oppCt.second;

				nitroRe = RobotEntity(robot);
				nitroAction = Action();
				nitroAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
				nitroAction.target_velocity_x = robot.velocity_x;
				nitroAction.target_velocity_y = Constants::Rules.MAX_ENTITY_SPEED;
				nitroAction.target_velocity_z = robot.velocity_z;
				nitroAction.use_nitro = true;
				nitroRe.Action = nitroAction;

				bool isCollision = simulate_robot_nitro_jump(
					nitroRe, oppId, 0, collisionTime, false);

				const int collisionTickDelta = 2;
				if (isCollision &&
					collisionTime * Constants::Rules.TICKS_PER_SECOND < oppCollisionTick - collisionTickDelta)//���� �� ������ �� ������� � �����
				{
					collisionTicks = int(collisionTime * Constants::Rules.TICKS_PER_SECOND) + 1;
					_actions[robot.id] = nitroAction;
					_nitroTicks[robot.id] = collisionTicks;
					return;
				}
			}							
		}
	}

	//if (!_isGoalPossible && _minOppCollisionTick != -1)
	//{
	//	//��������, ����� ������ ������� �� ���� ������ ����������
	//	for (int stop_nitro_tick = 59; stop_nitro_tick >= 2; --stop_nitro_tick)
	//	{
	//		auto nitroRe = RobotEntity(robot);
	//		Action nitroAction = Action();
	//		nitroAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
	//		nitroAction.target_velocity_x = robot.velocity_x;
	//		nitroAction.target_velocity_y = Constants::Rules.MAX_ENTITY_SPEED;
	//		nitroAction.target_velocity_z = robot.velocity_z;
	//		nitroAction.use_nitro = true;
	//		nitroRe.Action = nitroAction;

	//		std::vector<BallEntity> resBes;
	//		double collisionTime;
	//		bool isCollision = simulate_ball_nitro_jump(
	//			nitroRe, 0, resBes, collisionTime, true, stop_nitro_tick);

	//		if (isCollision && resBes[0].Velocity.Z > 0&&
	//			collisionTime * Constants::Rules.TICKS_PER_SECOND < _minOppCollisionTick)
	//		{
	//			auto collisionTicks = int(collisionTime * Constants::Rules.TICKS_PER_SECOND) + 1;
	//			_actions[robot.id] = nitroAction;
	//			_nitroTicks[robot.id] = collisionTicks;
	//			return;
	//		}
	//	}
	//}

	
	_actions[robot.id] = robotAction;	

}

model::Action MyStrategy::GetDefaultAction(const model::Robot & me, const Vector3D & defaultPos, int& runTicks)
{
	const auto targetVelocity = GetDefendPointTargetVelocity(me, defaultPos, runTicks);
	model::Action action = model::Action();
	action.target_velocity_x = targetVelocity.X;
	action.target_velocity_y = 0.0;
	action.target_velocity_z = targetVelocity.Z;
	action.jump_speed = 0.0;
	action.use_nitro = false;
	return action;
}

model::Action MyStrategy::GetNearestOppAttackAction(const model::Robot & me, int excludeId)
{
	const auto nearestOpp = get_my_gates_opp_robot(excludeId);
	const auto mePos = Helper::GetRobotPosition(me);
	auto oppPos = Helper::GetRobotPosition(nearestOpp);
	const double tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
	oppPos.X += nearestOpp.velocity_x * tickTime;
	oppPos.Z += nearestOpp.velocity_z * tickTime;
	const auto targetVelocity = Helper::GetTargetVelocity(
		mePos, oppPos, Constants::Rules.ROBOT_MAX_GROUND_SPEED);

	model::Action action = model::Action();
	action.target_velocity_x = targetVelocity.X;
	action.target_velocity_y = 0.0;
	action.target_velocity_z = targetVelocity.Z;
	action.jump_speed = 0.0;
	action.use_nitro = false;
	return action;
}

model::Action MyStrategy::GetMoveKeeperOrOppAction(const model::Robot & robot, int& resIndex)
{	
	Robot gk{};
	double maxZ = -50;
	for (auto &r : _robots)
	{
		if (r.is_teammate) continue;
		if (!r.touch) continue;
		if (r.z > maxZ)
		{
			maxZ = r.z;
			gk = r;
		}
	}

	if (abs(maxZ + 50) > EPS)
	{
		resIndex = 0;
		const auto mePos = Helper::GetRobotPosition(robot);
		auto oppPos = Helper::GetRobotPosition(gk);
		const double tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
		oppPos.X += gk.velocity_x * tickTime;
		oppPos.Z += gk.velocity_z * tickTime;
		const auto targetVelocity = Helper::GetTargetVelocity(mePos, oppPos, Constants::Rules.ROBOT_MAX_GROUND_SPEED);

		model::Action action = model::Action();
		action.target_velocity_x = targetVelocity.X;
		action.target_velocity_y = 0.0;
		action.target_velocity_z = targetVelocity.Z;
		action.jump_speed = 0.0;
		action.use_nitro = false;
		return action;
	}
	//���� �� ����������		
	resIndex = 1;
	return GetNearestOppAttackAction(robot, -1);		
	
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

	Simulator::Tick(bec, jumpRes, hitE, isGoalScored);
	

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

	//���������� ����� �� ����� - �� ��� ���� 2 ���������
	bool isGoalScored = false;
	/*Simulator::Update(robotEntity, ballEntity,
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
	Simulator::Update(robotEntity, ballEntity,
		1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
		(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
	*/

	//������� ����� ��������
		
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

			//��������� �� ����� ��������� ��������� ����
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
			if (myRobotExists && !res.at(0).IsCollided) return false;//����� ��� ����������
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
				if (!res.at(0).IsCollided)//��������� � ����� �� ����, ��� ��������� � ���/������
				{
					return false;
				}
				else //��� � ����-�� �������������� ���������. ������ ��������� ����� ������
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

		//������� ���, ��� �������� � �����
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
				//	return false;//�������� � ����� ���������� �� ����� �������������
				if (forbidNegativeVy && res.at(0).Velocity.Y < 0)
					return false;
				if (forbidDownStrike && res.at(0).Position.Y > beCopy.Position.Y)
					return false;
			}						

			//return true;//������� �� �������� �� �������� � �����
		}
		be = beCopy;
		bool isBeCollided = false;
		std::vector<bool> resIsCollided = std::vector<bool>();
		for (auto& re:res)
		{
			resIsCollided.push_back(false);
		}

		while (true)
		{
			Simulator::UpdateOnAir(
				be, res, 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
				hitE, isGoalScored, false);

			auto isCollisionDetected = false;

			for (int i =0 ;i < res.size(); ++i)
			{
				if (res[i].IsCollided)
				{
					isCollisionDetected = true;
					resIsCollided[i] = true;
					res[i].IsCollided = false;
				}
			}

			if (be.IsCollided)
			{
				isCollisionDetected = true;
				isBeCollided = true;
				be.IsCollided = false;
			}						
			
			if (!isCollisionDetected)
				break;
		}
		be.IsCollided = isBeCollided;
		for (int i = 0;i < res.size(); ++i)
			res[i].IsCollided = resIsCollided[i];
	}
	throw "WHY AM I HERE?";
}

bool MyStrategy::IsPenaltyArea(const Vector3D & position) const
{
	return  position.Z < 0;
}

double MyStrategy::GetVectorAngleToHorizontal(const Vector3D & v) const
{
	double horVector =
		sqrt(v.X*v.X + v.Z*v.Z);
	double angle = atan(v.Y / horVector);
	return angle;
}

//-1 - �� ����������, 0 - ���, 1 - ��
int MyStrategy::CompareDefenderBallEntities(const BallEntityContainer & b1, const BallEntityContainer & b2) const
{
	auto ballEntity1 = b1.ResBallEntity.Velocity.Z > 0 ? b1.ResBallEntity : b1.CollideBallEntity;
	auto ballEntity2 = b2.ResBallEntity.Velocity.Z > 0 ? b2.ResBallEntity : b2.CollideBallEntity;
	if (ballEntity1.Velocity.Z > 0 && ballEntity2.Velocity.Z > 0)
	{
		//return b1.ResBallEntity.Velocity.Z > b2.ResBallEntity.Velocity.Z ? -1 : 1;

		const double v1HorAngle = GetVectorAngleToHorizontal(ballEntity1.Velocity);
		const double v2HorAngle = GetVectorAngleToHorizontal(ballEntity2.Velocity);
		if (v1HorAngle > M_PI / 9 && v2HorAngle > M_PI / 9)
		{
			return ballEntity1.Velocity.Z > ballEntity2.Velocity.Z ? -1 : 1;
		}
		return ballEntity1.Velocity.Y > ballEntity2.Velocity.Y ? -1 : 1;
	}

	return ballEntity1.Velocity.Z > ballEntity2.Velocity.Z ? -1 : 1;

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
	const auto deltaTickTime = 5.0 / Constants::Rules.TICKS_PER_SECOND;
	if (_oppStrikeTime.has_value())//���� ���� �� ���� �� ��������� ���� ����� �����, �������� ��� �������, ��� ���� ������ 
	{
		if (bec2.collisionTime > _oppStrikeTime.value() - deltaTickTime)
		{
			return bec1.collisionTime < bec2.collisionTime ? -1 : 1;
		}
		if (bec1.collisionTime >= _oppStrikeTime.value() - deltaTickTime)
			return 1;
	}
	//��������� �� ���
	if (bec2.isGoalScored && !bec1.isGoalScored)
		return 1;
	if (bec1.isGoalScored && !bec2.isGoalScored)
		return -1;

	if (bec1.isGoalScored && bec2.isGoalScored)
	{
		//�������� �� ��� � ������
		if (bec1.NitroDest > bec2.NitroDest)
			return -1;
		if (bec1.NitroDest < bec2.NitroDest)
			return 1;

		//��������� �� ��� � ������� ������ �������� ���� � ������
		if (bec1.collisionsCount < bec2.collisionsCount)
			return -1;
		if (bec1.collisionsCount > bec2.collisionsCount)
			return 1;

		//��������� �� ��� � ������������ ��������� �� y
		return bec1.ResBallEntity.Velocity.Y > bec2.ResBallEntity.Velocity.Y ? -1 : 1;
	}

	//��� �������� �� ��������	

	//��������� �� ���� � ������
	if (bec1.NitroDest > 0 && bec2.NitroDest <= 0)
		return -1;
	if (bec1.NitroDest <= 0 && bec2.NitroDest > 0)
		return 1;

	return CompareDefenderBallEntities(bec1, bec2);
}

//��������� 2 ��������� �����, � ����� ������� ���� � ����� ������ �� �����
int MyStrategy::CompareDefenceNitroBeContainers(
	const BallEntityContainer& bec1, const BallEntityContainer& bec2,
	const RobotEntity& re1, const RobotEntity& re2) const
{
	//��������� �� ��� (��������, ������ ��������)
	if (bec1.isGoalScored && !bec2.isGoalScored)
		return -1;
	if (!bec1.isGoalScored && bec2.isGoalScored)
		return 1;

	//��������� �� ���� � �������������� ��������� ������ �� Y ����� ��������
	if (re1.Velocity.Y < 0 && re2.Velocity.Y > 0)
		return -1;
	if (re1.Velocity.Y > 0 && re2.Velocity.Y < 0)
		return -1;

	//��������� �� ���� � ������������� ��������� ���� �� Y ����� ��������
	if (bec1.ResBallEntity.Velocity.Y > 0 && bec2.ResBallEntity.Velocity.Y < 0)
		return -1;
	if (bec1.ResBallEntity.Velocity.Y < 0 && bec2.ResBallEntity.Velocity.Y > 0)
		return 1;

	//��������� �� ���� � ������� ��������� �� Z
	const auto bec1Vz = bec1.ResBallEntity.Velocity.Z > 0 ? bec1.ResBallEntity.Velocity.Z : bec1.CollideBallEntity.Velocity.Z;
	const auto bec2Vz = bec2.ResBallEntity.Velocity.Z > 0 ? bec2.ResBallEntity.Velocity.Z : bec2.CollideBallEntity.Velocity.Z;

	return bec1Vz > bec2Vz ? -1 : 1;
}

bool MyStrategy::IsGoalBallDirection2(
	const BallEntity & startBallEntity, int directionCoeff, bool considerBoardSide , double& goalTime, BallEntity& collideBallEntity, 
	bool isNitro, int& collisionsCount) const
{
	collisionsCount = 0;
	if (startBallEntity.Velocity.Z * directionCoeff <= 0) return false;
	if (abs(startBallEntity.Velocity.Z) < EPS) return false;
	
	const auto ballVelocityZDist = abs(startBallEntity.Velocity.Z) * BallMoveTicks * 1.0 / Constants::Rules.TICKS_PER_SECOND;
	const auto ballToGatesDist = abs(startBallEntity.Position.Z - (Constants::Rules.arena.depth / 2 + startBallEntity.Radius) * directionCoeff);
	if (ballVelocityZDist < ballToGatesDist) return false;//��� �� ��������� �� ����� �� 100 �����
	
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

	bool isFirstArenaCollision = true;
	for (int t = 1; t <= BallMoveTicks; ++t)
	{		
		ballEntity = SimulateTickBall(ballEntity, emptyJumpRes, isGoalScored, false);
		if (isGoalScored) {
			goalTime = t * 1.0 / Constants::Rules.TICKS_PER_SECOND;
			return true;
		}

		if (isFirstArenaCollision && ballEntity.IsArenaCollided)
		{
			isFirstArenaCollision = false;
			collideBallEntity = ballEntity;
		}

		if (directionCoeff == 1 && ballEntity.IsArenaCollided)
		{
			collisionsCount++;
			if (isNitro)
				return false;
			if (collisionsCount > MaxCollisionsToGoalStrike)
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

std::optional<Vector3D> MyStrategy::GetDefenderStrikePoint(int t,
	int startAttackTick,///��� ����������� ����� �������� t ������
	//int endAttackTick, //��� ����������� ����� �������� t ������ + 1
	BallEntityContainer& bestBecP, int& bestWaitT, int& bestMoveT, const StopContainer& stopContainer)
{
	std::optional<Vector3D> movePoint = std::nullopt;

	const BallEntity ballEntity = _ballEntities.at(t);
	//��� ����� ��������� �������� �� 4.8 �����
	auto yDist = ballEntity.Position.Y - Constants::Rules.BALL_RADIUS - Constants::Rules.ROBOT_MAX_RADIUS;
	const auto maxHeight = 4.8;
	const auto maxHeightTime = 0.5;
	if (yDist > maxHeight)
		return movePoint;

	auto vectorToBall = Vector3D(ballEntity.Position.X - _oppGates.X, 0, ballEntity.Position.Z - _oppGates.Z);
	auto length = vectorToBall.Length();
	vectorToBall.mult((length + Constants::Rules.BALL_RADIUS) / length);
	auto goToPoint = Vector3D(_oppGates.X + vectorToBall.X, Constants::Rules.ROBOT_RADIUS, _oppGates.Z + vectorToBall.Z);
	const auto time = t * 1.0 / Constants::Rules.TICKS_PER_SECOND;
	/*if (!CanGetToPoint(goToPoint, Helper::GetRobotPosition(robot), Helper::GetRobotVelocity(robot), time, true))
		return movePoint;*/

	double tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
	double microTickTime = tickTime / Constants::Rules.MICROTICKS_PER_TICK;

	//const auto deltaTicks = endAttackTick - startAttackTick;
	for (int waitT = 0; waitT < t; ++waitT)
	{
		const auto microTicks = waitT * Constants::Rules.MICROTICKS_PER_TICK;
		Vector3D waitPos;
		Vector3D waitVelocity;
		if (microTicks < stopContainer.stopMicroTicks)
		{
			waitPos = stopContainer.robotPosition + stopContainer.robotVelocity * (microTicks * microTickTime) +
				stopContainer.stopA * (microTickTime * microTicks *(microTicks + 1) / 2.0);
			waitVelocity = stopContainer.robotVelocity + stopContainer.stopA *  microTicks;
		}
		else
		{
			waitPos = Vector3D(stopContainer.stopPos);
			waitVelocity = Vector3D(0, 0, 0);
		}

		if (!IsInsideArenaPoint(waitPos))
			return movePoint;

		auto timeToJump = time - waitT * 1.0 / Constants::Rules.TICKS_PER_SECOND;
		if (!CanGetToPoint(goToPoint, waitPos, waitVelocity, timeToJump, false))
			return movePoint;//������ ����� ��� ������ - ����� �� �������

		auto curPos = Vector3D(waitPos);
		auto curVelocity = Vector3D(waitVelocity);

		bool isGettingCloser = false;

		for (int moveT = startAttackTick; moveT < t - waitT; ++moveT)
		{
			if (_ballEntities.at(moveT + waitT).Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)//�� � �������
				break;
					   
			timeToJump = time - (moveT + waitT) * 1.0 / Constants::Rules.TICKS_PER_SECOND;
			//if (timeToJump > maxHeightTime)continue; TODO: �������� �� ����

			if (moveT > 0)
			{
				PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
					curPos,
					goToPoint,
					curVelocity,
					1,
					1.0);

				if (!isGettingCloser)
				{
					if (Helper::GetLength2(pvContainer.Position, goToPoint) < Helper::GetLength2(waitPos, goToPoint))
					{
						isGettingCloser = true;
					}
				}
				else
				{
					if (Helper::GetLength2(waitPos, pvContainer.Position) > Helper::GetLength2(waitPos, goToPoint))
						return movePoint;// ��������� ������� �����. ������ ��� ��������������
				}
				curPos = pvContainer.Position;
				curVelocity = pvContainer.Velocity;
			}

			if (!IsInsideArenaPoint(curPos))
				return movePoint;
		
			if (!CanGetToPoint(goToPoint, curPos, curVelocity, timeToJump, false))
				continue;

			std::optional<double> curJumpCollisionT = std::nullopt;
			std::optional<BallEntity> collision_ball_entity = std::nullopt;
			BallEntity collideBallEntity;

			if (IsPenaltyArea(_ballEntities.at(moveT + waitT).Position) && //���� ��� ������ � ���� ������
				IsOkDefenderPosToJump(curPos, curVelocity,
				waitT + moveT,
				curJumpCollisionT, collision_ball_entity, collideBallEntity))
			{
				double goalTime = -1;
				BallEntity collideBallEntityAttack;
				int collisionsCount;
				const auto isGoal = IsGoalBallDirection2(collision_ball_entity.value(), 1, true, goalTime, collideBallEntityAttack, false, collisionsCount);
				const double curCollisionT = (moveT + waitT) * 1.0 / Constants::Rules.TICKS_PER_SECOND + curJumpCollisionT.value();
				const auto bec = BallEntityContainer(collision_ball_entity.value(), curCollisionT, isGoal, goalTime, collisionsCount, collideBallEntity, -1);

				if (movePoint == std::nullopt || CompareBeContainers(bec, bestBecP) < 0)
				{
					movePoint = goToPoint;
					bestBecP = bec;
					bestMoveT = moveT;
					bestWaitT = waitT;
				}
			}

		}
	}
	
	
	return movePoint;
}

bool MyStrategy::CanGetToPoint(const Vector3D& goToPoint, const Vector3D& robotPos, const Vector3D& robotVelocity, double time, bool canAccelerate)
{	
	auto goToPointDist = Vector3D(goToPoint.X - robotPos.X, goToPoint.Y - robotPos.Y, goToPoint.Z - robotPos.Z).Length();
	auto robotVelocityLength = robotVelocity.Length();

	double speedUpT = 0;
	if (canAccelerate)
	{
		speedUpT = (Constants::Rules.ROBOT_MAX_GROUND_SPEED - robotVelocityLength) / Constants::Rules.ROBOT_ACCELERATION;
		if (speedUpT > time)
			speedUpT = time;
	}

	auto speedUpS = robotVelocityLength * speedUpT + Constants::Rules.ROBOT_ACCELERATION * speedUpT * speedUpT / 2;
	if (speedUpS < goToPointDist - Constants::Rules.BALL_RADIUS - Constants::Rules.ROBOT_MAX_RADIUS)
	{
		auto maxSpeedT = time - speedUpT;
		auto maxSpeedS = Constants::Rules.ROBOT_MAX_GROUND_SPEED * maxSpeedT;
		if (speedUpS + maxSpeedS < goToPointDist - Constants::Rules.BALL_RADIUS - Constants::Rules.ROBOT_MAX_RADIUS)
			return false;
	}
	return true;
}

bool MyStrategy::IsOkDefenderPosToJump(
	const Vector3D & robotPosition, const Vector3D & robotVelocity,  
	int beforeTicks,
	std::optional<double>& jumpCollisionT, std::optional<BallEntity>& collisionBallEntity, BallEntity& collideBallEntity)
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
			if (_isNoCollisionMeGoalPossible) return false;//��� ������ ���� �� ������ - ��� ������� ��� ����� � ��� ������
			if (!_isMeGoalPossible) return false; //��� ������ ���� �� ������ - ������ ��� �� ����� � ��� ������
			continue;
		}
		beCur.IsCollided = false;
		beCur.IsArenaCollided = false;

		if (beCur.Position.Z < -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS)
			return false;

		//������� ��� �������� �������
		/*if (!IsPenaltyArea(beCur.Position, isDefender))
			return false;*/

		if (beCur.Velocity.Z < 0)
		{
			if (!_isMeGoalPossible) return false;
			BallEntity collideBallEntityCur;
			int collisionsCount;
			const auto isCollisionGoalPossible = IsGoalBallDirection2(beCur, -1, false, goalTime, collideBallEntityCur, false, collisionsCount);
			if (i==0)
			{
				collideBallEntity = collideBallEntityCur;
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
//		if (tBall > 0 && tJump > 0 && tBall <= BallMoveTicks) //tBall > BallMoveTicks ����� ����������, ���� ����� ����� ��� ����� � SetAttAction �� StartTick > 1
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
//				for (auto it = _defenderMovePoints.cbegin(); it != _defenderMovePoints.cend();)//������� ��� ��������� ���������
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

bool MyStrategy::IsInsideArenaPoint(const Vector3D& position)
{
	if (abs(position.X) > Constants::Rules.arena.width / 2 - Constants::Rules.arena.bottom_radius)
		return false;
	if (abs(position.Z) > Constants::Rules.arena.depth / 2 - Constants::Rules.arena.bottom_radius)
		if (abs(position.X) > Constants::Rules.arena.goal_width / 2 ||
			abs(position.Z) >
			Constants::Rules.arena.depth / 2 + Constants::Rules.arena.goal_depth - Constants::Rules.arena.bottom_radius)
			return false;
	return true;
}

model::Action MyStrategy::SetAttackerAction(const model::Robot & me,
	int startAttackTick,
	BallEntityContainer& bestBecP,
	bool& isOkBestBecP,
	int position)// -1 - ���, 0 - ��, 1 - ���
{
	isOkBestBecP = false;

	
	model::Action action = model::Action();

	const auto startPos = Helper::GetRobotPosition(me);
	const auto startVel = Helper::GetRobotVelocity(me);

	bool needUseDefenceNitro = position == -1 && me.nitro_amount > 0 &&
		(_lastMyCollisionTick == -1 || _isMeGoalPossible)
		/*&& (_minOppCollisionTick == -1 || 
			_minOppCollisionTick != -1 && (_meGoalScoringTick == -1 || _meGoalScoringTick != -1 && _meGoalScoringTick <= 30))*/
		&& (_defenderMovePoints.count(me.id) == 0 || !std::get<3>(_defenderMovePoints[me.id]).isGoalScored);
	const auto goalSearchTicks = 40;
	if (needUseDefenceNitro)
	{
		bool isDangerousBall = false;
		for (auto t = 0; t < goalSearchTicks; ++t)
		{
			if (_ballEntities[t].Position.Z < -Constants::Rules.arena.depth / 4 &&
				_ballEntities[t].Position.Z > -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS &&
				abs(_ballEntities[t].Position.X) < 3.0/8.0*Constants::Rules.arena.width &&
				_ballEntities[t].Position.Y > Constants::Rules.arena.goal_height / 2 && 
				_ballEntities[t].Position.Y < Constants::Rules.arena.goal_height)
			{
				isDangerousBall = true;
				break;
			}
		}
		if (!isDangerousBall)
			needUseDefenceNitro = false;
	}

	if (needUseDefenceNitro)
	{	
		Action nitroAction;
		RobotEntity bestRe;
		for (int t = 0; t < goalSearchTicks; ++t)
		{
			const auto targetBe = _ballEntities[t];
			Vector3D targetPos = Vector3D(targetBe.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
				targetBe.Position.Z);

			Vector3D curPos = Vector3D(startPos);
			Vector3D curVelocity = Vector3D(startVel);
			bool isGettingCloser = false;
			for (int moveT = 0; moveT <= goalSearchTicks; ++moveT)
			{				
				if (_meGoalScoringTick != -1 && moveT >= _meGoalScoringTick)
					break;
				if (moveT > 0)
				{
					PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
						curPos,
						targetPos,
						curVelocity,
						1,
						1.0);

					if (!isGettingCloser)
					{
						if (Helper::GetLength2(pvContainer.Position, targetPos) < Helper::GetLength2(startPos, targetPos))
						{
							isGettingCloser = true;
						}
					}
					else
					{
						if (Helper::GetLength2(startPos, pvContainer.Position) > Helper::GetLength2(startPos, targetPos))
							break;;// ��������� ������� �����. ������ ��� ��������������
					}
					curPos = pvContainer.Position;
					curVelocity = pvContainer.Velocity;
				}

				auto re = RobotEntity(curPos, curVelocity,
					Constants::Rules.ROBOT_MIN_RADIUS, true, Vector3D(0, 1, 0), me.nitro_amount);

				std::vector<BallEntity> resBes;
				double collisionTime;
				bool isCollision = simulate_ball_nitro_jump(re, moveT, resBes, collisionTime, true, -1);

				double goalTime;
				BallEntity collideBe;
				int collisions_count;
				bool isMeGoal = isCollision && IsGoalBallDirection2(resBes[0], -1, true, goalTime, collideBe,
					false, collisions_count);

				if (isCollision && resBes[0].Position.Z > -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS &&
					resBes[0].Velocity.Y > 0 
					&& !isMeGoal &&
					resBes[0].Velocity.Z > abs(resBes[0].Velocity.X) &&
					re.Velocity.Y < 0) //����� ���� ����� ��������
				{
					
					
					BallEntity attCollideBe;
					
					bool isGoal = IsGoalBallDirection2(resBes[0], 1, true, goalTime, attCollideBe, false, collisions_count);
					auto bec = BallEntityContainer(resBes[0], collisionTime, 
						isGoal, goalTime,collisions_count, collideBe, 1);

					if (!isOkBestBecP || CompareDefenceNitroBeContainers(bec, bestBecP, re, bestRe) < 0)
					{
						isOkBestBecP = true;
						bestBecP = bec;
						bestRe = re;
						if (moveT == 0)
						{
							//_nitroPosCur[me.id] = targetBe.Position;
							_nitroTicksCur[me.id] = int(collisionTime * Constants::Rules.TICKS_PER_SECOND) + 1;

							model::Action jumpAction = model::Action();
							jumpAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
							jumpAction.target_velocity_x = curVelocity.X;
							jumpAction.target_velocity_y = 0;
							jumpAction.target_velocity_z = curVelocity.Z;
							jumpAction.use_nitro = false;
							nitroAction = jumpAction;
						}
						else
						{
							auto const moveVelocity = Helper::GetTargetVelocity(startPos,
								targetPos,
								Constants::Rules.ROBOT_MAX_GROUND_SPEED);

							model::Action moveAction = model::Action();
							moveAction.jump_speed = 0;
							moveAction.target_velocity_x = moveVelocity.X;
							moveAction.target_velocity_y = 0;
							moveAction.target_velocity_z = moveVelocity.Z;
							moveAction.use_nitro = false;
							nitroAction = moveAction;
						}
					}
				}
			}
		}
		if (isOkBestBecP)
			return nitroAction;
	}

	Vector3D targetVelocity;
	RobotEntity robotEntity = RobotEntity(me);	

	bool isDefenderSavedPointOk;
	int bestWaitT=-1;
	int bestMoveT=-1;
	std::optional<Vector3D> movePoint =
		GetAttackerMovePoint(
			me, startAttackTick, isDefenderSavedPointOk, bestBecP, bestWaitT, bestMoveT, position);

	
	if (bestWaitT == 0 && bestMoveT == 0)
	{
		isOkBestBecP = true;
		targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, _ball.x, 0, _ball.z,
			Constants::Rules.ROBOT_MAX_GROUND_SPEED);
		action.target_velocity_x = targetVelocity.X;
		action.target_velocity_y = 0.0;
		action.target_velocity_z = targetVelocity.Z;
		action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
		action.use_nitro = false;

		return action;			
	}

	bool hasNearRobots = false;
	bool isStrikeZone = IsInsideArenaPoint(startPos);
	for (auto r:_robots)
	{
		if (r.id == me.id) continue;
		if ((me.x - r.x)*(me.x - r.x) + (me.y - r.y) * (me.y - r.y) + (me.z - r.z)*(me.z - r.z) <=
			(Constants::Rules.ROBOT_MAX_RADIUS + r.radius) * (Constants::Rules.ROBOT_MAX_RADIUS + r.radius))
		{
			hasNearRobots = true;
			break;
		}
	}

	if (!hasNearRobots && isStrikeZone && position >= 0 && startAttackTick == 0 &&
		me.nitro_amount > 0)
	{		

		for (int stop_nitro_tick = 59; stop_nitro_tick >= 2; --stop_nitro_tick)
		{
			auto re = RobotEntity(startPos, startVel,
				Constants::Rules.ROBOT_MIN_RADIUS, true, Vector3D(0, 1, 0), me.nitro_amount);

			std::vector<BallEntity> resBes;
			double collisionTime;
			bool isCollision = simulate_ball_nitro_jump(re, 0, resBes, collisionTime, true, stop_nitro_tick);

			double goalTime;
			BallEntity collideBallEntity;
			int collisionsCount;
			if (isCollision &&
				IsGoalBallDirection2(resBes[0], 1, false, goalTime, collideBallEntity, true, collisionsCount))
			{
				isOkBestBecP = true;
				bestBecP = BallEntityContainer(resBes[0], collisionTime, true, goalTime, collisionsCount, collideBallEntity, 1);
				
				_nitroTicksCur[me.id] = stop_nitro_tick - 1;

				model::Action jumpAction = model::Action();
				jumpAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
				jumpAction.target_velocity_x = startVel.X;
				jumpAction.target_velocity_y = 0;
				jumpAction.target_velocity_z = startVel.Z;
				jumpAction.use_nitro = false;
				return jumpAction;	

			}
		}
	}

	
	if (position >= 0 && me.z > 0 && startAttackTick == 0 &&
		(movePoint == std::nullopt || 
			_oppStrikeTime.has_value() && bestBecP.collisionTime > _oppStrikeTime.value()) &&
		me.nitro_amount > 0)
	{
		int finalTick = _minOppCollisionTick == -1 ? 50 : _minOppCollisionTick;


		bool isGot = false;
		BallEntityContainer nitroBestBec;
		Action nitroBestAction;
		for (int t = 0; t < finalTick; ++t)
		{
			const auto targetBe = _ballEntities[t];
			Vector3D targetPos = Vector3D(targetBe.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
				targetBe.Position.Z);

			bool isGettingCloser = false;
			Vector3D curPos = Vector3D(startPos);
			Vector3D curVelocity = Vector3D(startVel);

			for (int moveT = 0; moveT <= t; ++moveT)
			{
				if (moveT == 0 && (hasNearRobots || !isStrikeZone)) continue;
				if (moveT > 0)
				{
					PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
						curPos,
						targetPos,
						curVelocity,
						1,
						1.0);

					if (!isGettingCloser)
					{
						if (Helper::GetLength2(pvContainer.Position, targetPos) < Helper::GetLength2(startPos, targetPos))
						{
							isGettingCloser = true;
						}
					}
					else
					{
						if (Helper::GetLength2(startPos, pvContainer.Position) > Helper::GetLength2(startPos, targetPos))
							break;;// ��������� ������� �����. ������ ��� ��������������
					}
					curPos = pvContainer.Position;
					curVelocity = pvContainer.Velocity;
				}

				auto re = RobotEntity(curPos, curVelocity,
					Constants::Rules.ROBOT_MIN_RADIUS, true, Vector3D(0, 1, 0), me.nitro_amount);


				std::vector<BallEntity> resBes;
				double collisionTime;
				bool isCollision = simulate_ball_nitro_jump(re, moveT, resBes, collisionTime, true, -1);

				double goalTime;
				BallEntity collideBallEntity;
				int collisionsCount;
				if (isCollision && 					 
					IsGoalBallDirection2(resBes[1], 1, false, goalTime, collideBallEntity, true, collisionsCount) && 
					IsGoalBallDirection2(resBes[2], 1, false, goalTime, collideBallEntity, true, collisionsCount) &&
					IsGoalBallDirection2(resBes[0], 1, false, goalTime, collideBallEntity, true, collisionsCount) &&
					(!_isGoalPossible || (collisionTime + goalTime) * Constants::Rules.TICKS_PER_SECOND < _meGoalScoringTick))//��������� ����� ������� TODO
				{
					auto bec = BallEntityContainer(resBes[0], collisionTime, true, goalTime, collisionsCount, collideBallEntity, 1);

					if (!isGot || CompareBeContainers(bec, nitroBestBec))
					{
						isGot = true;
						nitroBestBec = bec;
						isOkBestBecP = true;

						if (moveT == 0)
						{
							//_nitroPosCur[me.id] = targetBe.Position;
							_nitroTicksCur[me.id] = int(collisionTime * Constants::Rules.TICKS_PER_SECOND) + 1;

							model::Action jumpAction = model::Action();
							jumpAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
							jumpAction.target_velocity_x = curVelocity.X;
							jumpAction.target_velocity_y = 0;
							jumpAction.target_velocity_z = curVelocity.Z;
							jumpAction.use_nitro = false;
							nitroBestAction = jumpAction;
						}
						else
						{
							auto const moveVelocity = Helper::GetTargetVelocity(startPos,
								targetPos,
								Constants::Rules.ROBOT_MAX_GROUND_SPEED);

							model::Action moveAction = model::Action();
							moveAction.jump_speed = 0;
							moveAction.target_velocity_x = moveVelocity.X;
							moveAction.target_velocity_y = 0;
							moveAction.target_velocity_z = moveVelocity.Z;
							moveAction.use_nitro = false;
							nitroBestAction = moveAction;
						}

					}

					
					
					
				}
			}
		}

		if (isGot)
		{
			return nitroBestAction;
		}
	}

	if (position >= 0 && startAttackTick == 0 && me.z > 0 &&
		(movePoint == std::nullopt ||
			_oppStrikeTime.has_value() && bestBecP.collisionTime > _oppStrikeTime.value()) &&
		!_isGoalPossible && _isNoCollisionGoalPossible && me.nitro_amount > 0)
	{
		bool isGot = false;
		BallEntityContainer nitroBestBec;
		double minCollisionTime = std::numeric_limits<double>::max();
		Action nitroBestAction;

		for (auto & oppCt : _oppBallCollisionTicks)
		{
			const int oppId = oppCt.first;
			const int oppCollisionTick = oppCt.second;



			for (int t = 0; t < oppCollisionTick; ++t)
			{
				RobotEntity targetRe = GetRobotEntity(t, oppId);

				Vector3D targetPos = Vector3D(targetRe.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
					targetRe.Position.Z);

				bool isGettingCloser = false;
				Vector3D curPos = Vector3D(startPos);
				Vector3D curVelocity = Vector3D(startVel);

				for (int moveT = 0; moveT <= t; ++moveT)
				{
					if (moveT == 0 && (hasNearRobots || !isStrikeZone)) continue;
					if (moveT > 0)
					{
						PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
							curPos,
							targetPos,
							curVelocity,
							1,
							1.0);

						if (!isGettingCloser)
						{
							if (Helper::GetLength2(pvContainer.Position, targetPos) < Helper::GetLength2(startPos, targetPos))
							{
								isGettingCloser = true;
							}
						}
						else
						{
							if (Helper::GetLength2(startPos, pvContainer.Position) > Helper::GetLength2(startPos, targetPos))
								break;;// ��������� ������� �����. ������ ��� ��������������
						}
						curPos = pvContainer.Position;
						curVelocity = pvContainer.Velocity;
					}

					auto re = RobotEntity(curPos, curVelocity,
						Constants::Rules.ROBOT_MIN_RADIUS, true, Vector3D(0, 1, 0), me.nitro_amount);


					double collisionTime;
					bool isCollision = simulate_robot_nitro_jump(re, oppId, moveT, collisionTime, true);

					const int collisionTickDelta = 2;
					if (isCollision &&
						collisionTime * Constants::Rules.TICKS_PER_SECOND < oppCollisionTick - collisionTickDelta)//���� �� ������ �� ������� � �����
					{
						const auto goalTime = _noCollisionGoalScoringTick * 1.0 / Constants::Rules.TICKS_PER_SECOND;

						if (!isGot || collisionTime < minCollisionTime)
						{
							isGot = true;
							auto bec = BallEntityContainer(_ballEntities[0], collisionTime, true, goalTime, 0, BallEntity(), 0);
							nitroBestBec = bec;
							minCollisionTime = collisionTime;

							if (moveT == 0)
							{
								_nitroTicksCur[me.id] = int(collisionTime * Constants::Rules.TICKS_PER_SECOND) + 1;

								model::Action jumpAction = model::Action();
								jumpAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
								jumpAction.target_velocity_x = curVelocity.X;
								jumpAction.target_velocity_y = 0;
								jumpAction.target_velocity_z = curVelocity.Z;
								jumpAction.use_nitro = false;
								nitroBestAction = jumpAction;
							}
							else
							{
								auto const moveVelocity = Helper::GetTargetVelocity(startPos,
									targetPos,
									Constants::Rules.ROBOT_MAX_GROUND_SPEED);

								model::Action moveAction = model::Action();
								moveAction.jump_speed = 0;
								moveAction.target_velocity_x = moveVelocity.X;
								moveAction.target_velocity_y = 0;
								moveAction.target_velocity_z = moveVelocity.Z;
								moveAction.use_nitro = false;
								nitroBestAction = moveAction;
							}

						}
					}
				}
			}
		}
		if (isGot)
		{
			isOkBestBecP = true;
			bestBecP = nitroBestBec;
			return nitroBestAction;
		}
	}


	if (movePoint == std::nullopt)
	{
		isOkBestBecP = false;
		return action;
	}

	isOkBestBecP = true;
	
	targetVelocity = Helper::GetTargetVelocity(me.x, 0, me.z, movePoint.value().X, 0, movePoint.value().Z,
		Constants::Rules.ROBOT_MAX_GROUND_SPEED);		

	action.target_velocity_x = targetVelocity.X;
	action.target_velocity_y = 0.0;
	action.target_velocity_z = targetVelocity.Z;
	action.jump_speed = 0.0;
	action.use_nitro = false;
	return action;	
}

bool MyStrategy::IsOkPosToMove(const Vector3D & mePos, const model::Robot & robot,
	int t, int directionCoeff, std::optional<double>& collisionT, std::optional<BallEntity>& bestBallEntity, double& goalTime, int& collisionsCount)
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
		return false; // ��������� ������� �����. ������ ��� ��������������

	if (!IsInsideArenaPoint(pvContainer.Position))
		return false;

	if (Helper::GetLength(_ballEntities.at(t).Position, pvContainer.Position) < 
		Constants::Rules.ROBOT_MIN_RADIUS + Constants::Rules.BALL_RADIUS)
		return false; //��������� ��� ��� �������� � �����

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
		IsOkPosToJump(robotE, t, jumpCollisionT, ball_entity, goalTime, collisionsCount) :
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

double MyStrategy::GetMoveRadius(const model::Robot & robot, int t)
{
	const auto be = _ballEntities.at(t);

	const auto jumpRobotVelocity = 15.0;
	const auto collisionT = (be.Position.Y - Constants::Rules.ROBOT_MIN_RADIUS) / (jumpRobotVelocity - be.Velocity.Y);
	if (collisionT < 0)
		return _moveSphereRadius;
	   
	const auto x = be.Position.X + be.Velocity.X * collisionT;
	const auto z = be.Position.Z + be.Velocity.Z * collisionT;

	const auto lengthToLine = Helper::GetDistToLine(robot.x, robot.z, be.Position.X, be.Position.Z, x, z);
	return lengthToLine + _moveSphereRadius;
}

std::optional<Vector3D> MyStrategy::GetAttackerMovePoint(const model::Robot & robot,
	int startAttackTick,
	bool& isDefenderSavedPointOk, BallEntityContainer& bestBecP, int& bestWaitT, int& bestMoveT, int position)
{
	/*if (_isSamePosition &&
		_defenderMovePoints.count(robot.id) > 0 && std::get<1>(_defenderMovePoints[robot.id]) > 0)
	{
		isDefenderSavedPointOk = true;
		bestWaitT = std::get<1>(_defenderMovePoints[robot.id]);
		bestMoveT = std::get<2>(_defenderMovePoints[robot.id]);
		bestBecP = std::get<3>(_defenderMovePoints[robot.id]);
		return Helper::GetRobotPosition(robot);
	}

	if (_defenderMovePoints.count(robot.id) > 0)
		_defenderMovePoints.erase(robot.id);*/


	isDefenderSavedPointOk = false;
	std::optional<Vector3D> movePoint = std::nullopt;
	const auto tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;

	//if (_defenderMovePoints.count(robot.id) > 0)
	//{
	//	auto times = _defenderMovePoints[robot.id];
	//	const auto tBall = std::get<0>(times);
	//	const auto tWait = std::get<1>(times);
	//	const auto tJump = std::get<2>(times);

	//	if (tBall > 0 && tWait >= 0 && tJump > 0 && tBall <= BallMoveTicks) //tBall > BallMoveTicks ����� ����������, ���� ����� ����� ��� ����� � SetAttAction �� StartTick > 1
	//	{
	//		bool isPassedBy = false;
	//		int bestMoveT;
	//		int bestWaitT;
	//		BallEntityContainer dmpBecP;
	//		const auto defenderMovePoint = GetDefenderStrikePoint(robot, tBall, tJump, tJump + 1, dmpBecP, bestMoveT, bestWaitT);

	//		if (defenderMovePoint != std::nullopt && dmpBecP.isGoalScored && (!_oppStrikeTime.has_value() || dmpBecP.collisionTime < _oppStrikeTime.value() - tickTime))
	//		{
	//			isDefenderSavedPointOk = true;
	//			bestBecP = dmpBecP;

	//			const auto be = _ballEntities.at(tBall);
	//			for (auto it = _defenderMovePoints.cbegin(); it != _defenderMovePoints.cend();)//������� ��� ��������� ���������
	//			{
	//				if ((*it).first != robot.id)
	//					_defenderMovePoints.erase(it);
	//				++it;
	//			}

	//			return defenderMovePoint;
	//		}
	//	}
	//	if (tJump != 0)//��� ������ - ��� ���������� �����
	//		_defenderMovePoints.erase(robot.id);
	//}

	//int bestT = -1;
	bool isResOk = false;

	const auto robotPos = Helper::GetRobotPosition(robot);
	const auto robotVel = Helper::GetRobotVelocity(robot);
	const auto stopContainer = GetStopContainer(robotPos, robotVel);

	const auto deltaAngle = M_PI / 180 * 2;
	bool isVectorSet = false;
	Vector3D curRobotBallVector;

	const auto isWaiting = _defenderMovePoints.count(robot.id) > 0 && std::get<1>(_defenderMovePoints[robot.id]) > 0;

	bool gotDmp = false;
	std::tuple<int, int, int, BallEntityContainer> curDmp;

	for (int t = startAttackTick; t <= startAttackTick + BallMoveTicks; ++t)
	{
		if (_goalScoringTick >= 0 && t >= _goalScoringTick)
		{
			if (gotDmp)			
				_defenderMovePoints[robot.id] = curDmp;			
			else			
				_defenderMovePoints.erase(robot.id);			
			return movePoint;
		}
		if (startAttackTick == 0 && _meGoalScoringTick >= 0 && t >= _meGoalScoringTick)
		{
			if (gotDmp)
				_defenderMovePoints[robot.id] = curDmp;
			else
				_defenderMovePoints.erase(robot.id);
			return movePoint;
		}
		if (_ballEntities.count(t) == 0)
		{
			if (gotDmp)
				_defenderMovePoints[robot.id] = curDmp;
			else
				_defenderMovePoints.erase(robot.id);
			return movePoint;
		}

		auto ballEntity = _ballEntities.at(t);
		
		if (IsPenaltyArea(ballEntity.Position))//�������� ����� ���������
		{
			if (robot.z > 0 && position == 1) continue;//��� �� ����������, ���� �� �� ����� ��������

			const auto isBestTime = _defenderMovePoints.count(robot.id) > 0 && std::get<0>(_defenderMovePoints[robot.id]) == t;

			const auto robotBallVector = Vector3D(ballEntity.Position.X - robotPos.X, 0, ballEntity.Position.Z - robotPos.Z);
			const auto angle = curRobotBallVector.angleTo(robotBallVector);
 			if (!isWaiting && !isBestTime && isVectorSet && abs(angle) < deltaAngle && movePoint.has_value())
				continue;

			isVectorSet = true;
			curRobotBallVector = robotBallVector;

			std::optional<double> curCollisionT = std::nullopt;
			bool isPassedBy = false;

			int curBestWaitT;
			int curBestMoveT;
			
			BallEntityContainer becP;
			auto defenderMovePoint = GetDefenderStrikePoint(t, startAttackTick,becP, curBestWaitT, curBestMoveT, stopContainer);
			if (defenderMovePoint == std::nullopt) continue;

			if (!isResOk || CompareBeContainers(becP, bestBecP) < 0)
			{
				bestWaitT = curBestWaitT;
				bestMoveT = curBestMoveT;

				isResOk = true;
				movePoint = bestWaitT == 0 ? defenderMovePoint : Helper::GetRobotPosition(robot);
				bestBecP = becP;

				if (startAttackTick == 0)
				{
					gotDmp = true;
					curDmp = std::make_tuple(t, bestWaitT, bestMoveT, becP);
				}
			}			
		}
		else
		{
			if (position == -1) continue;//��� �� �������

			std::optional<double> jumpCollisionT = std::nullopt;
			std::optional<BallEntity> curBallEntity = std::nullopt;
			double goalTime;
			int collisionsCount;

			if (t == 0)
			{
				auto robotEntity = RobotEntity(robot);
				if (IsOkPosToJump(robotEntity, 0, jumpCollisionT, curBallEntity, goalTime, collisionsCount))
				{
					BallEntityContainer bec = BallEntityContainer(
						curBallEntity.value(), jumpCollisionT.value(), true, goalTime, collisionsCount, BallEntity(), -1);
					bestWaitT = 0;
					bestMoveT = 0;
					isResOk = true;
					movePoint = robotEntity.Position;
					bestBecP = bec;
				}
				continue;
			}

			
			const auto attackMovePoint = GetAttackerStrikePoint(robot, t, 1, jumpCollisionT, curBallEntity, goalTime, collisionsCount);
			
			if (attackMovePoint == std::nullopt)
				continue;

			const auto curCollisionT = t * 1.0 / Constants::Rules.TICKS_PER_SECOND + jumpCollisionT.value();
			BallEntityContainer bec = BallEntityContainer(curBallEntity.value(), curCollisionT, true, goalTime, collisionsCount, BallEntity(), -1);
			if (!isResOk || CompareBeContainers(bec, bestBecP) < 0)
			{
				bestWaitT = -1;
				bestMoveT = -1;
				_beforeStrikePoints[robot.id] = std::pair<int, Vector3D>(t, attackMovePoint.value());
				isResOk = true;
				movePoint = attackMovePoint;
				bestBecP = bec;
			}			
		}
	}

	if (gotDmp)
		_defenderMovePoints[robot.id] = curDmp;
	else
		_defenderMovePoints.erase(robot.id);

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
	std::optional<double>& collisionT, std::optional<BallEntity>& bestBallEntity, double& goalTime, int& collisionsCount)
{
	const BallEntity ballEntity = _ballEntities.at(t);
	if (_beforeStrikePoints.count(robot.id) > 0)
	{
		if (_beforeStrikePoints[robot.id].first == t)
		{
			if (IsOkPosToMove(_beforeStrikePoints[robot.id].second, robot, t, directionCoeff, collisionT, bestBallEntity, goalTime, collisionsCount))
			{
				return _beforeStrikePoints[robot.id].second;
			}			
			_beforeStrikePoints.erase(robot.id);			
		}
	}



	//���, ����� ����� ��������� ������ ��������. TODO: ��������������
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
	double radius = GetMoveRadius(robot, t);
	double tmp = radius / sqrt(1 + div * div);
	double x1 = ballEntity.Position.X - tmp;
	double z1 = ballEntity.Position.Z - (robot.x - ballEntity.Position.X) * (x1 - ballEntity.Position.X) /
		(robot.z - ballEntity.Position.Z);
	Vector3D rVector1 = Vector3D(x1 - ballEntity.Position.X, 0.0, z1 - ballEntity.Position.Z);

	double x2 = ballEntity.Position.X + tmp;
	double z2 = ballEntity.Position.Z - (robot.x - ballEntity.Position.X) * (x2 - ballEntity.Position.X) /
		(robot.z - ballEntity.Position.Z);
	Vector3D rVector2 = Vector3D(x2 - ballEntity.Position.X, 0.0, z2 - ballEntity.Position.Z);

	Vector3D mePos = Vector3D(ballEntity.Position.X, Constants::Rules.ROBOT_MIN_RADIUS, ballEntity.Position.Z);
//TODO: ����� ���� ����� �������� ������ goalTime
	if (IsOkPosToMove(mePos, robot, t, directionCoeff, collisionT, bestBallEntity, goalTime, collisionsCount))
	{
		return mePos;
	}

	for (int i = 1; i <= StrikeSphereStepsCount; ++i)
	{
		Vector3D rVectorMult1 = rVector1 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos1 = Vector3D(ballEntity.Position.X + rVectorMult1.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult1.Z);
		if (IsOkPosToMove(mePos1, robot, t, directionCoeff, collisionT, bestBallEntity, goalTime, collisionsCount))
		{
			return mePos1;
		}

		Vector3D rVectorMult2 = rVector2 * (i * 1.0 / StrikeSphereStepsCount);
		Vector3D mePos2 = Vector3D(ballEntity.Position.X + rVectorMult2.X, Constants::Rules.ROBOT_MIN_RADIUS,
			ballEntity.Position.Z + rVectorMult2.Z);
		if (IsOkPosToMove(mePos2, robot, t, directionCoeff, collisionT, bestBallEntity, goalTime, collisionsCount))
		{
			
			return mePos2;
		}
	}

	return std::nullopt;
}

bool MyStrategy::IsOkPosToJump(
	RobotEntity & robotEntity,
	int beforeTicks,
	std::optional<double>& collisionT, std::optional<BallEntity>& bestBallEntity, double& goalTime, int& collisionsCount)
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
			if (!_isNoCollisionGoalPossible) return false; //�� ������ ���� �� ������ - ��� ������� ��� �� ����� � �������
			if (_isGoalPossible) return false;//��� ������ ���� �� ������ - ��� ����� � ������
			// TODO: ����� �� �������. ���� ���������, ��� ��� ������� � ������. ����� ���� ���� �� ������, ����� ��� �������� � �����
		}
		beCur.IsCollided = false;
		beCur.IsArenaCollided = false;

		//������� � �������� �������
		/*if (IsPenaltyArea(beCur.Position, false))
			return false;*/

		//double angle = GetVectorAngleToHorizontal(beCur.Velocity);
		//if (angle * directionCoeff > M_PI / 3) return false; //TODO: �� ���� ��� ������� ����� � �����������

		if (_isGoalPossible && beCur.Velocity.Z < ballEntity.Velocity.Z)
			return false;//�� ���� � �������� ��������� �� z

		double ballFlyTime = 0;
		BallEntity collideBallEntity;
		int collisionCountCur;
		if (!IsGoalBallDirection2(beCur, directionCoeff, false, ballFlyTime, collideBallEntity, false, collisionCountCur)) return false;
		if (i==0)
		{
			goalTime = ballFlyTime;
			collisionsCount = collisionCountCur;
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
		BallEntity collideBallEntity;
		int collisionsCount;
		if (!IsGoalBallDirection2(beCur, directionCoeff, false, goalTime, collideBallEntity, false, collisionsCount)) return false;
	}	
	return true;
}

Vector3D MyStrategy::GetDefendPointTargetVelocity(const model::Robot & robot, const Vector3D& position, int& runTicks)
{
	const auto mtTime = 1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK;

	auto robotPos = Helper::GetRobotPosition(robot);
	auto robotVelocity = Helper::GetRobotVelocity(robot);

	Vector3D p1 = robotPos;
	Vector3D v1 = robotVelocity;

	int ticks = 0;
	int stopTicks = 0;
	while (true)
	{

		if (v1.Length2() < Constants::Rules.ROBOT_MAX_GROUND_SPEED * Constants::Rules.ROBOT_MAX_GROUND_SPEED)
		{
			const auto tv1Length = Constants::Rules.ROBOT_ACCELERATION * mtTime;
			auto tvc = Helper::GetTargetVelocity(p1, position, tv1Length);

			p1 = p1 + v1 * mtTime * Constants::Rules.MICROTICKS_PER_TICK +
				tvc * (mtTime * Constants::Rules.MICROTICKS_PER_TICK * (Constants::Rules.MICROTICKS_PER_TICK + 1) / 2.0);
			v1 = v1 + tvc * Constants::Rules.MICROTICKS_PER_TICK;
		}
		else
		{
			p1 = p1 + v1 * mtTime * Constants::Rules.MICROTICKS_PER_TICK;
		}


		auto stopContainer = GetStopContainer(p1, v1);
		auto stopDist = Helper::GetLength2(robotPos, stopContainer.stopPos);
		auto needDist = Helper::GetLength2(robotPos, position);
		if (stopDist - needDist > -EPS)
		{
			stopTicks = int(stopContainer.stopMicroTicks * 1.0 / Constants::Rules.MICROTICKS_PER_TICK);
			break;
		}
		ticks++;
	}

	runTicks = ticks + stopTicks;

	if (ticks == 0)//��� ���� ���������
	{
		auto targetVelocity = Vector3D(0, 0, 0);
		return targetVelocity;
	}

	auto deltaPos = position - robotPos;
	deltaPos.Normalize();
	auto targetVelocity = deltaPos * Constants::Rules.ROBOT_MAX_GROUND_SPEED;
	return targetVelocity;
}

std::optional<double> MyStrategy::GetOppStrikeTime(const std::vector<model::Robot>& oppRobots)
{
	std::optional<double> minTime = std::nullopt;

	for (int t = 1; t <= BallMoveTicks; ++t)
	{
		auto be = _ballEntities.at(t);
		auto ballHorPos = Vector3D(be.Position.X, Constants::Rules.ROBOT_RADIUS, be.Position.Z);
		for (auto& robot : oppRobots)
		{
			if (!robot.touch) continue;

			auto robotPos = Helper::GetRobotPosition(robot);
			auto robotVelocity = Helper::GetRobotVelocity(robot);

			auto pvContainer = Simulator::GetRobotPVContainer(robotPos, ballHorPos, robotVelocity, t, 1);
			if (pvContainer.IsPassedBy || Helper::GetLength2(pvContainer.Position, ballHorPos) <
				(Constants::Rules.BALL_RADIUS + Constants::Rules.ROBOT_MAX_RADIUS) * (Constants::Rules.BALL_RADIUS + Constants::Rules.ROBOT_MAX_RADIUS))
			{
				double time = t * 1.0 / Constants::Rules.TICKS_PER_SECOND;
				if (minTime == std::nullopt || time < minTime.value())
				{
					minTime = time;
				}
				break;
			}
		}
	}
	return minTime;	
}

model::Robot MyStrategy::get_my_gates_opp_robot(int excludeId)
{
	auto min_dist = std::numeric_limits<double>::max();
	Robot nearest_robot{};
	for (Robot r : _robots)
	{
		if (r.is_teammate) continue;
		if (r.id == excludeId) continue;
		
		auto const dist = (_myGates.X - r.x)*(_myGates.X - r.x) + (_myGates.Z - r.z)*(_myGates.Z - r.z);
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest_robot = r;
		}	

	}
	return nearest_robot;

}

bool MyStrategy::IsSamePosition()
{
	bool isSameBall =
		abs(_ball.x - _ballEntities[1].Position.X) < EPS &&
		abs(_ball.y - _ballEntities[1].Position.Y) < EPS &&
		abs(_ball.z - _ballEntities[1].Position.Z) < EPS &&
		abs(_ball.velocity_x - _ballEntities[1].Velocity.X) < EPS &&
		abs(_ball.velocity_y - _ballEntities[1].Velocity.Y) < EPS &&
		abs(_ball.velocity_z - _ballEntities[1].Velocity.Z) < EPS;
	if (!isSameBall) return false;


	for (auto & robot : _robots)
	{
		if (robot.touch)
			continue;

		bool gotSame = false;
		for (auto & re : _robotEntities[1])
		{					
			bool isSameRobot =
				abs(robot.x - re.Position.X) < EPS &&
				abs(robot.y - re.Position.Y) < EPS &&
				abs(robot.z - re.Position.Z) < EPS &&
				abs(robot.velocity_x - re.Velocity.X) < EPS &&
				abs(robot.velocity_y - re.Velocity.Y) < EPS &&
				abs(robot.velocity_z - re.Velocity.Z) < EPS;
			if (isSameRobot)
			{
				gotSame = true;
				break;
			}		
		}
		if (!gotSame)
			return false;
	}
	return true;
}



void MyStrategy::InitBallEntities()
{
	_isSamePosition = IsSamePosition();
	
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
			if (_nitroTicks.count(robot.id) > 0 && _nitroTicks[robot.id] > 0)
			{
				re.Action.target_velocity_x = robot.velocity_x;
				re.Action.target_velocity_x = Constants::Rules.MAX_ENTITY_SPEED;
				re.Action.target_velocity_z = robot.velocity_z;
				re.Action.use_nitro = true;
			}
			_robotEntities[0].push_back(re);
		}
	}	
	
	bool gotOppCollision = false;

	for (auto t = 1; t <= BallMoveTicks; ++t)
	{
		std::vector<RobotEntity> jumpResCur = std::vector<RobotEntity>();
		for (auto & re: _robotEntities.at(t-1))
		{
			auto reCopy = RobotEntity(re);
			if (_nitroTicks.count(reCopy.Id) > 0 && t >= _nitroTicks[reCopy.Id])
			{
				reCopy.Action.target_velocity_x = 0;
				reCopy.Action.target_velocity_x = 0;
				reCopy.Action.target_velocity_z = 0;
				reCopy.Action.use_nitro = false;
			}
			jumpResCur.push_back(reCopy);
		}

		ball_entity = SimulateTickBall(ball_entity, jumpResCur, isGoalScored, false);		

		if (!_isGoalPossible && ball_entity.Position.Z > Constants::Rules.arena.depth / 2 + ball_entity.Radius)
		{
			_isGoalPossible = true;
		}

		if (!_isMeGoalPossible && ball_entity.Position.Z < -Constants::Rules.arena.depth / 2 - ball_entity.Radius)
		{
			_isMeGoalPossible = true;
			_meGoalScoringTick = t;
		}		

		bool isOppCollided = false;
		for (auto& re : jumpResCur)//TODO: ������� ������ �������� � �����, � �� ���� � ������
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
					_lastMyCollisionTick = t;
				}
				else
				{
					isOppCollided = true;
					if (ball_entity.IsCollided)//�������� ����� � �����
					{						
						_oppBallCollisionTicks[re.Id] = t;
						if (_minOppCollisionTick == -1)
							_minOppCollisionTick = t;
					}
				}
				re.IsCollided = false;
			}
		}

		ball_entity.IsCollided = false;
		ball_entity.IsArenaCollided = false;

		_ballEntities[t] = ball_entity;

		if (!gotOppCollision)
		{
			if (isOppCollided)
				gotOppCollision = true;
			else
			{
				if (!_isNoCollisionGoalPossible && ball_entity.Position.Z > Constants::Rules.arena.depth / 2 + ball_entity.Radius)
				{
					_isNoCollisionGoalPossible = true;
					_noCollisionGoalScoringTick = t;
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
	_lastSimulationTick = BallMoveTicks;

	int addTicks = _lastMyCollisionTick == -1 ? 0 : _lastMyCollisionTick + AttackerAddTicks;

	if (addTicks > 0)//������������
	{
		for (auto t = BallMoveTicks + 1; t <= BallMoveTicks + addTicks; ++t)
		{
			std::vector<RobotEntity> jumpResCur = std::vector<RobotEntity>();
			for (auto & re : _robotEntities.at(t - 1))
			{
				jumpResCur.push_back(re);
			}

			ball_entity = SimulateTickBall(ball_entity, jumpResCur, isGoalScored, false);
			

			if (!_isGoalPossible && ball_entity.Position.Z > Constants::Rules.arena.depth / 2 + ball_entity.Radius)
			{
				_isGoalPossible = true;
			}

			if (!_isMeGoalPossible && ball_entity.Position.Z < -Constants::Rules.arena.depth / 2 - ball_entity.Radius)
			{
				_isMeGoalPossible = true;
				_meGoalScoringTick = t;
			}

			bool isOppCollided = false;
			for (auto& re : jumpResCur)//TODO: ������� ������ �������� � �����, � �� ���� � ������
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
						_lastMyCollisionTick = t;
					}
					else
					{
						isOppCollided = true;						
					}
					re.IsCollided = false;
				}
			}

			ball_entity.IsCollided = false;
			ball_entity.IsArenaCollided = false;

			_ballEntities[t] = ball_entity;

			if (!gotOppCollision)
			{
				if (isOppCollided)
					gotOppCollision = true;
				else
				{
					if (!_isNoCollisionGoalPossible && ball_entity.Position.Z > Constants::Rules.arena.depth / 2 + ball_entity.Radius)
					{
						_isNoCollisionGoalPossible = true;
						_noCollisionGoalScoringTick = t;
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
		_lastSimulationTick = BallMoveTicks + addTicks;
	}


	//���� ���� �������� � �������, ������� ��� ��� ��� �������
	if (gotOppCollision)
	{
		ball_entity = BallEntity(_ball);
		std::vector<RobotEntity> noRes = std::vector<RobotEntity>();
		for (auto t = 1; t <= BallMoveTicks + addTicks; ++t)
		{
			ball_entity = SimulateTickBall(ball_entity, noRes, isGoalScored, true);

			if (!_isNoCollisionGoalPossible && ball_entity.Position.Z > Constants::Rules.arena.depth / 2 + ball_entity.Radius)
			{
				_isNoCollisionGoalPossible = true;
				_noCollisionGoalScoringTick = t;
			}

			if (!_isNoCollisionMeGoalPossible && ball_entity.Position.Z < -Constants::Rules.arena.depth / 2 - ball_entity.Radius)
			{
				_isNoCollisionMeGoalPossible = true;
			}
		}
	}

	
	for (auto t = 1; t <= BallMoveTicks + addTicks; ++t)
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

int MyStrategy::UpdateBallEntities(double collisionTime, const Vector3D& afterCollisionBallVelocity, bool isGoal)
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

	for (auto i = 1; i <= BallMoveTicks + AttackerAddTicks; ++i)
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

		//TODO: ������� �����
		if (_goalScoringTick == -1 && ballEntity.Position.Z > Constants::Rules.arena.depth / 2 + Constants::Rules.BALL_RADIUS)
			_goalScoringTick = afterCollisionTick + i;

		const auto be = _ballEntities.at(afterCollisionTick + i);
		_drawSpheres.push_back(Sphere(
			be.Position.X,
			be.Position.Y,
			be.Position.Z,
			2, 0, 1, isGoal ? 0 : 1, 0.25
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
	_lastSimulationTick = afterCollisionTick + BallMoveTicks + AttackerAddTicks;

	return afterCollisionTick;
}

StopContainer MyStrategy::GetStopContainer(const Vector3D& robotPosition, const Vector3D& robotVelocity) const
{
	auto robotVelocityLength = robotVelocity.Length();
	if (robotVelocityLength < EPS)
	{
		const auto stopA = Vector3D(0, 0, 0);
		return StopContainer(robotPosition, robotVelocity, 0, robotPosition, stopA);
	}

	double tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
	double microTickTime = tickTime / Constants::Rules.MICROTICKS_PER_TICK;
	const auto tv1Length = Constants::Rules.ROBOT_ACCELERATION * microTickTime;
	auto stopA = robotVelocity * (1.0 / robotVelocityLength * (-tv1Length));

	auto stopMts = int(robotVelocityLength / stopA.Length());

	auto stopPos = robotPosition + robotVelocity * (stopMts * microTickTime) +
		stopA * (microTickTime * stopMts * (stopMts + 1) / 2.0);

	return StopContainer(robotPosition, robotVelocity, stopMts, stopPos, stopA);
}

std::optional<model::NitroPack> MyStrategy::get_nearest_nitro_pack(const Robot& robot, const model::Game& game)
{
	if (robot.nitro_amount > Constants::Rules.MAX_NITRO_AMOUNT * 0.75)
		return std::nullopt;

	auto minDist = std::numeric_limits<double>::max();
	std::optional<NitroPack> res = std::nullopt;
	for (const auto& np : game.nitro_packs)
	{
		if (!np.alive)
			continue;

		//if (_gotNitros.find(np) != _gotNitros.end())
		//	continue;

		if (np.z * robot.z < 0)
			continue; //���� ������ �� ����� ��������

		const auto dist2 = (np.x - robot.x)*(np.x - robot.x) + (np.z - robot.z)*(np.z - robot.z);
		if (dist2 < minDist)
		{
			minDist = dist2;
			res = np;
		}
	}
	return res;
}

bool MyStrategy::simulate_ball_nitro_jump(
	RobotEntity& re, int startTick, std::vector<BallEntity>& resBes, double& collisionTime, bool isJump, int stopNitroTick)
{	
	int t = startTick;
	auto curBe = _ballEntities[t];

	double startDx = re.Position.X - curBe.Position.X;
	double startDy = re.Position.Y - curBe.Position.Y;
	double startDz = re.Position.Z - curBe.Position.Z;

	const auto tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
	const auto mictoTickTime = tickTime / Constants::Rules.MICROTICKS_PER_TICK;
	bool isGoalScored;

	//_drawSpheres.emplace_back(re.Position.X, re.Position.Y, re.Position.Z, 1, 1, 0, 0, 0.5);

	const auto constVy = isJump ? NitroVy : re.Velocity.Y;

	if (isJump)
	{
		re.Position.Y = NitroStartY;
		re.Velocity.Y = constVy;
	}

	const auto collisionDist = Constants::Rules.BALL_RADIUS + Constants::Rules.ROBOT_MAX_RADIUS;
	const auto collisionDist2 = collisionDist * collisionDist;
	auto jumpT = 0;

	resBes = std::vector<BallEntity>();
	auto nitro = re.Nitro;

	bool isNitroStop = false;
	while (_ballEntities.count(t+1) > 0 && jumpT + 1 < 100)
	{
		t++;
		jumpT++;

		curBe = BallEntity(_ballEntities[t]);

		re.Position.X += re.Velocity.X * tickTime;
		re.Position.Z += re.Velocity.Z * tickTime;

		if (isNitroStop || (!isNitroStop && stopNitroTick != -1 && t > stopNitroTick))
		{
			isNitroStop = true;

			re.Position.Y += re.Velocity.Y * tickTime - Constants::Rules.GRAVITY * tickTime * tickTime / 2.0;
			re.Velocity.Y -= Constants::Rules.GRAVITY * tickTime;
			if (re.Velocity.Y < 0)
				return false; //�� ���� �� ������

		}
		else if (!isJump || jumpT > 1)
		{			
			re.Position.Y += constVy * tickTime;
			nitro -= MtNitroLoss * Constants::Rules.MICROTICKS_PER_TICK;
		}

		if (re.Position.Y > Constants::Rules.arena.height - re.Radius)
			return false;

		if (curBe.Position.Y - re.Position.Y > collisionDist)//��� �� �������� ������ ������
		{
			/*if ((reX - curBe.Position.X) * startDx < 0 ||
				(reZ - curBe.Position.Z) * startDz < 0)
				return false;*/
			if (nitro < 0)
			{
				nitro = 0;
				isNitroStop = true;
			}
			continue;
		}
		if (re.Position.Y - curBe.Position.Y > collisionDist)//��������� ���� ����
		{
			return false;
		}

		//���������� ������ ��� ��������
		auto dist2 = (re.Position.X - curBe.Position.X) * (re.Position.X - curBe.Position.X) +
			(re.Position.Y - curBe.Position.Y) * (re.Position.Y - curBe.Position.Y) +
			(re.Position.Z - curBe.Position.Z) * (re.Position.Z - curBe.Position.Z);
		if (dist2 > collisionDist2)
		{
			if (nitro < 0) return false;
			continue; //�������� ��� �� ����
		}

		//��������� ��������
		int c = 0;
		while (dist2 < collisionDist2)
		{
			c++;
			re.Position.X -= re.Velocity.X * mictoTickTime;
			if (isNitroStop)
			{
				re.Position.Y = re.Position.Y - re.Velocity.Y * mictoTickTime + Constants::Rules.GRAVITY * mictoTickTime * mictoTickTime / 2.0;
				re.Velocity.Y += Constants::Rules.GRAVITY * mictoTickTime;
			}
			else
			{
				re.Position.Y -= constVy * mictoTickTime;
				nitro += MtNitroLoss;
			}
			re.Position.Z -= re.Velocity.Z * mictoTickTime;

			curBe.Position.X -= curBe.Velocity.X * mictoTickTime;
			curBe.Position.Y = curBe.Position.Y - curBe.Velocity.Y * mictoTickTime + Constants::Rules.GRAVITY * mictoTickTime * mictoTickTime / 2.0;
			curBe.Position.Z -= curBe.Velocity.Z * mictoTickTime;
			curBe.Velocity.Y += Constants::Rules.GRAVITY * mictoTickTime;

			dist2 = (re.Position.X - curBe.Position.X) * (re.Position.X - curBe.Position.X) +
				(re.Position.Y - curBe.Position.Y) * (re.Position.Y - curBe.Position.Y) +
				(re.Position.Z - curBe.Position.Z) * (re.Position.Z - curBe.Position.Z);
			
		}

		if (nitro < 0)
			return false;

		//���������� �������� ��������
		re.Radius = Constants::Rules.ROBOT_MAX_RADIUS;

		model::Action jumpNitroAction = model::Action();
		jumpNitroAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
		if (isNitroStop)
		{
			jumpNitroAction.target_velocity_x = 0;
			jumpNitroAction.target_velocity_y = 0;
			jumpNitroAction.target_velocity_z = 0;
			jumpNitroAction.use_nitro = false;
		}
		else
		{
			jumpNitroAction.target_velocity_x = re.Velocity.X;
			jumpNitroAction.target_velocity_y = Constants::Rules.MAX_ENTITY_SPEED;
			jumpNitroAction.target_velocity_z = re.Velocity.Z;
			jumpNitroAction.use_nitro = true;
		}
		re.Action = jumpNitroAction;

		collisionTime = (t - c / 100.0) * 1.0 / Constants::Rules.TICKS_PER_SECOND;
		RobotEntity resRe;
		for (int i = 0; i < 3; ++i)
		{
			auto reCopy = RobotEntity(re);
			auto resBe = BallEntity(curBe);

			bool gotCollision = false;
			while (true)
			{
				Simulator::Update(reCopy, resBe, mictoTickTime, _allHitEs[i], isGoalScored);
				if (resBe.IsCollided)
				{
					gotCollision = true;
					resBe.IsCollided = false;
				}
				else
					break;
			}
			if (!gotCollision)
				return false;
			resBes.push_back(resBe);

			if (i == 0)
				resRe = reCopy;
		}
		re = resRe;
		return true;		
	}
	return false;
}

bool MyStrategy::simulate_robot_nitro_jump(
	RobotEntity & re, int oppRobotId, int startTick, double & collisionTime, bool isJump)
{
	int t = startTick;
	const auto tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
	const auto mictoTickTime = tickTime / Constants::Rules.MICROTICKS_PER_TICK;
	bool isGoalScored;

	//_drawSpheres.emplace_back(re.Position.X, re.Position.Y, re.Position.Z, 1, 1, 0, 0, 0.5);

	auto reX = re.Position.X;
	auto reZ = re.Position.Z;

	auto reY = isJump ? NitroStartY : re.Position.Y;

	bool isGot;
	auto curRe = GetRobotById(oppRobotId, 0, isGot);
	if (!isGot) return false;

	const auto collisionDist = curRe.Radius + Constants::Rules.ROBOT_MAX_RADIUS;
	const auto collisionDist2 = collisionDist * collisionDist;
	auto jumpT = 0;

	auto nitro = re.Nitro;
	while (_robotEntities.count(t + 1) > 0 && jumpT + 1 < 60)
	{
		t++;
		jumpT++;

		curRe = GetRobotById(oppRobotId, t, isGot);
		if (!isGot) return false;

		reX += re.Velocity.X * tickTime;
		reZ += re.Velocity.Z * tickTime;
		if (!isJump || jumpT > 1)
		{
			reY += NitroVy * tickTime;
			nitro -= MtNitroLoss * Constants::Rules.MICROTICKS_PER_TICK;
		}

		if (curRe.Position.Y - reY > collisionDist)//��� �� �������� ������ ������
		{
			/*if ((reX - curBe.Position.X) * startDx < 0 ||
				(reZ - curBe.Position.Z) * startDz < 0)
				return false;*/
			if (nitro < 0) return false;
			continue;
		}
		if (reY - curRe.Position.Y > collisionDist)//��������� ���� ������
		{
			return false;
		}

		//���������� ������ ��� ��������
		auto dist2 = (reX - curRe.Position.X) * (reX - curRe.Position.X) +
			(reY - curRe.Position.Y) * (reY - curRe.Position.Y) +
			(reZ - curRe.Position.Z) * (reZ - curRe.Position.Z);
		if (dist2 > collisionDist2)
		{
			if (nitro < 0) return false;
			continue; //�������� ��� �� ����
		}

		//��������� ��������
		int c = 0;
		while (dist2 < collisionDist2)
		{
			c++;
			reX -= re.Velocity.X * mictoTickTime;
			reY -= NitroVy * mictoTickTime;
			reZ -= re.Velocity.Z * mictoTickTime;

			curRe.Position.X -= curRe.Velocity.X * mictoTickTime;
			curRe.Position.Y = curRe.Position.Y - curRe.Velocity.Y * mictoTickTime + Constants::Rules.GRAVITY * mictoTickTime * mictoTickTime / 2.0;
			curRe.Position.Z -= curRe.Velocity.Z * mictoTickTime;
			curRe.Velocity.Y += Constants::Rules.GRAVITY * mictoTickTime;

			dist2 = (reX - curRe.Position.X) * (reX - curRe.Position.X) +
				(reY - curRe.Position.Y) * (reY - curRe.Position.Y) +
				(reZ - curRe.Position.Z) * (reZ - curRe.Position.Z);

			nitro += MtNitroLoss;
		}

		if (nitro < 0)
			return false;

		//���������� �������� ��������
		re.Position.X = reX;
		re.Position.Y = reY;
		re.Position.Z = reZ;
		re.Velocity.Y = NitroVy;
		re.Radius = Constants::Rules.ROBOT_MAX_RADIUS;

		model::Action jumpNitroAction = model::Action();
		jumpNitroAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
		jumpNitroAction.target_velocity_x = re.Velocity.X;
		jumpNitroAction.target_velocity_y = Constants::Rules.MAX_ENTITY_SPEED;
		jumpNitroAction.target_velocity_z = re.Velocity.Z;
		jumpNitroAction.use_nitro = true;
		re.Action = jumpNitroAction;

		collisionTime = (t - c / 100.0) * 1.0 / Constants::Rules.TICKS_PER_SECOND;

		auto curBe = BallEntity(_ballEntities.at(t));
		auto jumpRes = std::vector<RobotEntity>();
		jumpRes.push_back(re);
		jumpRes.push_back(curRe);
		for (int j = 0; j < 2; ++j)
		{
			Simulator::UpdateOnAir(curBe, jumpRes, mictoTickTime, _averageHitE[0], isGoalScored, false);
			if (jumpRes[1].IsCollided)
				break;
		}
		if (!jumpRes[1].IsCollided)
			return false;
		curRe.IsCollided = false;				
		return true;
	}
	return false;
}

RobotEntity MyStrategy::GetRobotEntity(int tick, int id)
{
	for (auto& re : _robotEntities.at(tick))
	{
		if (re.Id == id)
			return re;
	}
	throw "NO ROBOT FOUND";
}

model::Robot MyStrategy::GetRobotById(int id)
{
	for (auto& r : _robots)
	{
		if (r.id == id)
		{
			return r;
		}
	}
	throw "NOT FOUND";
}

RobotEntity MyStrategy::GetRobotById(int id, int tick, bool & isGot)
{
	isGot = false;
	RobotEntity re;
	if (_robotEntities.count(tick) == 0)
		return re;
	for (auto& r:_robotEntities[tick])
	{
		if (r.Id == id)
		{
			isGot = true;
			return r;
		}
	}
	return re;
}

model::NitroPack MyStrategy::GetNitroById(Game game, int id)
{
	for (auto& np : game.nitro_packs)
	{
		if (np.id == id)
		{
			return np;
		}
	}
	throw "NOT FOUND";
}

bool MyStrategy::IsSafoToCollectNitro()
{
	for (int i = 0; i <= BallMoveTicks;++i)
	{
		if (_ballEntities[i].Position.Z < EPS)
			return false;
	}
	return true;
}

model::Action MyStrategy::GetSaveGatesAction(const model::Robot & robot, bool& canBeSaved)
{
	canBeSaved = false;
	const int goalSearchTicks = 40;
	const Vector3D startPos = Helper::GetRobotPosition(robot);
	const Vector3D startVel = Helper::GetRobotVelocity(robot);

	Action nitroAction;
	BallEntityContainer bestBec;
	RobotEntity bestRe;

	for (int t = 0; t < goalSearchTicks; ++t)
	{
		const auto targetBe = _ballEntities[t];
		Vector3D targetPos = Vector3D(targetBe.Position.X, Constants::Rules.ROBOT_MIN_RADIUS,
			targetBe.Position.Z);

		Vector3D curPos = Vector3D(startPos);
		Vector3D curVelocity = Vector3D(startVel);
		bool isGettingCloser = false;
		for (int moveT = 0; moveT <= t; ++moveT)
		{
			if (_meGoalScoringTick != -1 && moveT >= _meGoalScoringTick)
				break;
			if (moveT > 0)
			{
				PositionVelocityContainer pvContainer = Simulator::GetRobotPVContainer(
					curPos,
					targetPos,
					curVelocity,
					1,
					1.0);

				if (!isGettingCloser)
				{
					if (Helper::GetLength2(pvContainer.Position, targetPos) < Helper::GetLength2(startPos, targetPos))
					{
						isGettingCloser = true;
					}
				}
				else
				{
					if (Helper::GetLength2(startPos, pvContainer.Position) > Helper::GetLength2(startPos, targetPos))
						break;;// ��������� ������� �����. ������ ��� ��������������
				}
				curPos = pvContainer.Position;
				curVelocity = pvContainer.Velocity;
			}

			auto re = RobotEntity(curPos, curVelocity,
				Constants::Rules.ROBOT_MIN_RADIUS, true, Vector3D(0, 1, 0), robot.nitro_amount);

			std::vector<BallEntity> resBes;
			double collisionTime;
			bool isCollision = simulate_ball_nitro_jump(re, moveT, resBes, collisionTime, true, -1);

			double goalTime;
			BallEntity collideBallEntity;
			int collisionCount;
			if (isCollision && resBes[0].Position.Z > -Constants::Rules.arena.depth / 2 - Constants::Rules.BALL_RADIUS &&
				collisionTime * Constants::Rules.TICKS_PER_SECOND < _meGoalScoringTick &&
				!IsGoalBallDirection2(resBes[0], -1, true, goalTime, collideBallEntity, false, collisionCount)) //����� ���� ����� ��������
			{
				BallEntity attCollideBe;
				int collisions_count;
				bool isGoal = IsGoalBallDirection2(resBes[0], 1, true, goalTime, attCollideBe, false, collisions_count);
				auto bec = BallEntityContainer(resBes[0], collisionTime,
					isGoal, goalTime, collisions_count, collideBallEntity, 1);

				if (!canBeSaved || CompareDefenceNitroBeContainers(bec, bestBec, re, bestRe) < 0)
				{
					canBeSaved = true;
					bestBec = bec;
					bestRe = re;
					if (moveT == 0)
					{
						//_nitroPosCur[me.id] = targetBe.Position;
						_nitroTicksCur[robot.id] = int(collisionTime * Constants::Rules.TICKS_PER_SECOND) + 1;

						model::Action jumpAction = model::Action();
						jumpAction.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;
						jumpAction.target_velocity_x = curVelocity.X;
						jumpAction.target_velocity_y = 0;
						jumpAction.target_velocity_z = curVelocity.Z;
						jumpAction.use_nitro = false;
						nitroAction = jumpAction;
					}
					else
					{
						auto const moveVelocity = Helper::GetTargetVelocity(startPos,
							targetPos,
							Constants::Rules.ROBOT_MAX_GROUND_SPEED);

						model::Action moveAction = model::Action();
						moveAction.jump_speed = 0;
						moveAction.target_velocity_x = moveVelocity.X;
						moveAction.target_velocity_y = 0;
						moveAction.target_velocity_z = moveVelocity.Z;
						moveAction.use_nitro = false;
						nitroAction = moveAction;
					}
				}
			}
		}
	}
	return nitroAction;
	
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