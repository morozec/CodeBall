#include "stdafx.h"
#include "CppUnitTest.h"
#include "../model/Ball.h"
#include "../model/Robot.h"
#include "../MyStrategy.h"
#include <vector>
#include "../Helper.h"
#include "../DanCalculator.h"
#include "../Simulator.h"
#include "../BallEntityContainer.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestProject
{		


	TEST_CLASS(UnitTest1)
	{
	private:
		model::Rules get_rules()
		{
			model::Arena arena = model::Arena();
			arena.width = 60;
			arena.height = 20;
			arena.depth = 80;
			arena.bottom_radius = 3;
			arena.top_radius = 7;
			arena.corner_radius = 13;
			arena.goal_top_radius = 3;
			arena.goal_width = 30;
			arena.goal_height = 10;
			arena.goal_depth = 10;
			arena.goal_side_radius = 1;

			model::Rules rules = model::Rules();
			rules.arena = arena;
			rules.max_tick_count = 18000;
			rules.seed = 91;
			rules.team_size = 2;
			rules.ROBOT_MIN_RADIUS = 1;
			rules.ROBOT_MAX_RADIUS = 1.05;
			rules.ROBOT_MAX_JUMP_SPEED = 15;
			rules.ROBOT_ACCELERATION = 100;
			rules.ROBOT_NITRO_ACCELERATION = 30;
			rules.ROBOT_MAX_GROUND_SPEED = 30;
			rules.ROBOT_ARENA_E = 0;
			rules.ROBOT_RADIUS = 1;
			rules.ROBOT_MASS = 2;
			rules.TICKS_PER_SECOND = 60;
			rules.MICROTICKS_PER_TICK = 100;
			rules.RESET_TICKS = 120;
			rules.BALL_ARENA_E = 0.7;
			rules.BALL_RADIUS = 2;
			rules.BALL_MASS = 1;
			rules.MIN_HIT_E = 0.4;
			rules.MAX_HIT_E = 0.4;
			rules.MAX_ENTITY_SPEED = 100;
			rules.MAX_NITRO_AMOUNT = 100;
			rules.START_NITRO_AMOUNT = 50;
			rules.NITRO_POINT_VELOCITY_CHANGE = 0.6;
			rules.NITRO_PACK_X = 20;
			rules.NITRO_PACK_Y = 1;
			rules.NITRO_PACK_Z = 30;
			rules.NITRO_PACK_RADIUS = 0.5;
			rules.NITRO_PACK_AMOUNT = 100;
			rules.NITRO_PACK_RESPAWN_TICKS = 600;
			rules.GRAVITY = 30;

			return rules;
		}

	public:
		TEST_METHOD(TestMethod1)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);


			model::Ball ball = model::Ball();
			ball.x = 4.926;
			ball.y = 8.197;
			ball.z = -35.912;
			ball.velocity_x = 1.074;
			ball.velocity_y = -20.792;
			ball.velocity_z = -29.682;
			ball.radius = 2.0;

			model::Robot robot = model::Robot();
			robot.x = 4.509;
			robot.y = 1.0;
			robot.z = -33.524;
			robot.velocity_x = 3.112;
			robot.velocity_y = 0.0;
			robot.velocity_z = -29.838;
			robot.touch = true;
			robot.touch_normal_x = 0.0;
			robot.touch_normal_y = 1.0;
			robot.touch_normal_z = 0.0;
			
			std::vector<model::Robot> oppRobots = std::vector<model::Robot>();
			oppRobots.push_back(robot);

						

			//model::Robot nbr = MyStrategy::get_nearest_my_gates_opp_robot(ball, oppRobots);
			
			//auto time = ms.GetOppStrikeTime(oppRobots);
		}

		TEST_METHOD(CompareVelocities)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball0{};
			ball0.x = 0;
			ball0.y = 5;
			ball0.z = 0;			
			ball0.radius = Constants::Rules.BALL_RADIUS;
			auto be0 = BallEntity(ball0);
			be0.Velocity = Vector3D(-15.2748, 11.62825, 17.63613);
			auto bec0 = BallEntityContainer(be0, 0, false, 0, 0, BallEntity(), -1);

			model::Ball ball1{};
			ball1.x = 0;
			ball1.y = 5;
			ball1.z = 0;
			ball1.radius = Constants::Rules.BALL_RADIUS;
			auto be1 = BallEntity(ball1);
			be1.Velocity = Vector3D(-30.9335, 16.85286, 29.78007);
			auto bec1 = BallEntityContainer(be1, 0, false, 0, 0, BallEntity(), -1);

			int isB2GoalDirection = -1;
			const auto compare = ms.CompareDefenderBallEntities(bec0, bec1);
			Assert::AreEqual(compare, 1);
		}

		TEST_METHOD(IsOkPosToJump)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = 25.04312;
			ball.y = 5.94034109968811;
			ball.z = -1.04324010642284;
			ball.velocity_x = 19.8656723361905;
			ball.velocity_y = 13.4748612612109;
			ball.velocity_z = 18.8094609108391;
			ball.radius = Constants::Rules.BALL_RADIUS;
			BallEntity be = BallEntity(ball);

			model::Robot robot{};
			robot.x = 22.6698490919545;
			robot.y = 1;
			robot.z = -3.31379489618211;
			robot.velocity_x = 21.6770975071995;
			robot.velocity_y = 0;
			robot.velocity_z = 20.7389354515453;
			auto re = RobotEntity(robot);

			std::optional<double> collisionT;
			std::optional<Vector3D> bestBallVelocity;
			/*const auto isOk = ms.IsOkPosToJump(re, 0, collisionT, bestBallVelocity);
			Assert::AreEqual(isOk, false);*/
		}

		TEST_METHOD(simulate_jump_start)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = 25.04312;
			ball.y = 5.94034109968811;
			ball.z = -1.04324010642284;
			ball.velocity_x = 19.8656723361905;
			ball.velocity_y = 13.4748612612109;
			ball.velocity_z = 18.8094609108391;
			ball.radius = Constants::Rules.BALL_RADIUS;
			BallEntity be = BallEntity(ball);

			model::Robot robot{};
			robot.x = 10;
			robot.y = 1;
			robot.z = 10;
			robot.velocity_x = -5;
			robot.velocity_y = 0;
			robot.velocity_z = 7;
			robot.radius = 1.0;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;

			auto re0 = RobotEntity(robot);
			re0.Action.jump_speed = 15;
			re0.Action.target_velocity_x = 5;
			re0.Action.target_velocity_z = 8;
			auto re1 = RobotEntity(robot);
			re1.Action.jump_speed = 15;
			re1.Action.target_velocity_x = 5;
			re1.Action.target_velocity_z = 8;

			bool isGoalScored;

			Simulator::Update(re0, be,
				1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
				0.45, isGoalScored);
			Simulator::Update(re0, be,
				1.0 / Constants::Rules.TICKS_PER_SECOND / Constants::Rules.MICROTICKS_PER_TICK,
				0.45, isGoalScored);

			Simulator::simulate_jump_start(re1);

			Assert::AreEqual(re0.Position.X, re1.Position.X, 1E-3);
			Assert::AreEqual(re0.Position.Y, re1.Position.Y, 1E-3);
			Assert::AreEqual(re0.Position.Z, re1.Position.Z, 1E-3);
			Assert::AreEqual(re0.Velocity.X, re1.Velocity.X, 1E-3);
			Assert::AreEqual(re0.Velocity.Y, re1.Velocity.Y, 1E-3);
			Assert::AreEqual(re0.Velocity.Z, re1.Velocity.Z, 1E-3);
			Assert::AreEqual(re0.Touch, re1.Touch);
		}

		TEST_METHOD(NeedStrike)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -9.253241587585697;
			ball.y = 4.093252991335528;
			ball.z = -1.7096336072805016;
			ball.velocity_x = 10.175199453371446;
			ball.velocity_y = -15.575798063898935;
			ball.velocity_z = 34.23866188231188;
			ball.radius = Constants::Rules.BALL_RADIUS;
			BallEntity be = BallEntity(ball);


			model::Robot robot{};
			robot.x = -8.369401220826916;
			robot.y = 1;
			robot.z = -2.8542756283268527;
			robot.velocity_x = 6.143441401413759;
			robot.velocity_y = 0;
			robot.velocity_z = 28.92256848426171;
			robot.radius = 1.0;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;

			model::Robot oppRobot{};
			oppRobot.x = -7.276036974390679;
			oppRobot.y = 1.2931415295836233;
			oppRobot.z = 3.978474612601338;
			oppRobot.velocity_x = -9.047866141652767;
			oppRobot.velocity_y = 14.48835027779566;
			oppRobot.velocity_z = -28.19239336152584;
			oppRobot.radius = 1.05;
			oppRobot.touch = false;
			oppRobot.touch_normal_x = 0;
			oppRobot.touch_normal_y = 0;
			oppRobot.touch_normal_z = 0;

			auto robots = std::vector<model::Robot>();
			robots.push_back(robot);
			robots.push_back(oppRobot);

			ms._ball = ball;
			ms._robots = robots;

			ms.InitBallEntities();

			BallEntityContainer bec;
			bool isOkBestBec;
			bool isDribbling;
			int dribblingTicks;
			ms.SetAttackerAction(
				robot, 1, bec, isOkBestBec, 0);
		}


		TEST_METHOD(StrangeJump)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -6.451210955598594;
			ball.y = 7.662059597569974;
			ball.z = 10.90590807634312;
			ball.velocity_x = 2.1431655874084505;
			ball.velocity_y = -12.755472706034233;
			ball.velocity_z = -25.96738559270208;
			ball.radius = Constants::Rules.BALL_RADIUS;
			BallEntity be = BallEntity(ball);


			model::Robot robot{};
			robot.x = -6.678818894476233;
			robot.y = 1;
			robot.z = 0.6800190363448757;
			robot.velocity_x = 3.1272286157453557;
			robot.velocity_y = 0;
			robot.velocity_z = 17.774938765996396;
			robot.radius = 1.0;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;

			model::Robot oppRobot{};
			oppRobot.x = -7.006357476854922;
			oppRobot.y = 4.69214794352452;
			oppRobot.z = 11.346472403495573;
			oppRobot.velocity_x = -13.587511166661262;
			oppRobot.velocity_y = 2.4971529256281206;
			oppRobot.velocity_z = -26.73010056705816;
			oppRobot.radius = 1.05;
			oppRobot.touch = false;
			oppRobot.touch_normal_x = 0;
			oppRobot.touch_normal_y = 0;
			oppRobot.touch_normal_z = 0;

			auto robots = std::vector<model::Robot>();
			robots.push_back(robot);
			robots.push_back(oppRobot);

			ms._ball = ball;
			ms._robots = robots;

			ms.InitBallEntities();

			BallEntityContainer bec;
			bool isOkBestBec;
			bool isDribbling;
			int dribblingTicks;
			auto action =  ms.SetAttackerAction(robot, 1, bec, isOkBestBec, 0);
			Assert::AreEqual(action.jump_speed, 0, 1E-3);
		}

		TEST_METHOD(Collision)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -6.451210955598594;
			ball.y = 7.662059597569974;
			ball.z = 10.90590807634312;
			ball.velocity_x = 2.1431655874084505;
			ball.velocity_y = -12.755472706034233;
			ball.velocity_z = -25.96738559270208;
			ball.radius = Constants::Rules.BALL_RADIUS;
			auto be = BallEntity(ball);


			model::Robot oppRobot{};
			oppRobot.x = -7.006357476854922;
			oppRobot.y = 4.69214794352452;
			oppRobot.z = 11.346472403495573;
			oppRobot.velocity_x = -13.587511166661262;
			oppRobot.velocity_y = 2.4971529256281206;
			oppRobot.velocity_z = -26.73010056705816;
			oppRobot.radius = 1.05;
			oppRobot.touch = false;
			oppRobot.touch_normal_x = 0;
			oppRobot.touch_normal_y = 0;
			oppRobot.touch_normal_z = 0;
			auto re = RobotEntity(oppRobot);
			re.Action.jump_speed = 15;

			Simulator::Tick(re, be);

			Assert::IsTrue(re.IsCollided);


		}

		TEST_METHOD(NeedScore)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -1.2047930015571386;
			ball.y = 8.098431555908238;
			ball.z = 37.915747297642696;
			ball.velocity_x = -25.536370627631648;
			ball.velocity_y = -2.00097693402524;
			ball.velocity_z = -0.13117537789750067;
			ball.radius = Constants::Rules.BALL_RADIUS;
			

			model::Robot robot{};
			robot.x = -0.18029641308209987;
			robot.y = 1;
			robot.z = 36.89840854486751;
			robot.velocity_x = -29.952858853498796;
			robot.velocity_y = 0;
			robot.velocity_z = -1.6811444025956992;
			robot.radius = 1.0;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			auto re = RobotEntity(robot);

			ms._goalScoringTick = -1;
			auto robots = std::vector<model::Robot>();
			robots.push_back(robot);
			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			std::optional<double> ct;
			bool isStrikeDefender;
			auto bestBe = BallEntity(ball);
			bool isSavedPointOk;
			double goalTime = 0;
			BallEntityContainer bec;
			int bestWaitT;
			int bestMoveT;
			auto mp = ms.GetAttackerMovePoint(robot, 1, isSavedPointOk, bec, bestWaitT, bestMoveT, 0);
			Assert::IsFalse(mp == std::nullopt);

			auto jumpRe = RobotEntity(robot);
			std::optional<BallEntity> optionalBestBe;
			int collisionsCount;
			auto isOkToJump = ms.IsOkPosToJump(jumpRe, 0, ct, optionalBestBe, goalTime, collisionsCount);
			Assert::IsFalse(isOkToJump);

			bool isOkBestBec;
			bool isDribbling;
			int dribblingTicks;
			auto action = ms.SetAttackerAction(robot, 1, bec, isOkBestBec, 0);		
			re.Action = action;
			for (auto & bsp : ms._beforeStrikePoints)
			{
				ms._beforeStrikePoints[bsp.first].first--;
			}
			
			auto be = BallEntity(ball);

			double deltaTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
			bool isGoalScored = false;
			bool isJumping = false;
			int counter = 0;

			while (!isJumping)
			{
				counter++;
				for (int i = 0; i < Constants::Rules.MICROTICKS_PER_TICK; ++i)
				{
					Simulator::Update(re, be, deltaTime / Constants::Rules.MICROTICKS_PER_TICK,
						(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
				}

				auto robot2 = model::Robot();
				robot2.x = re.Position.X;
				robot2.y = re.Position.Y;
				robot2.z = re.Position.Z;
				robot2.velocity_x = re.Velocity.X;
				robot2.velocity_y = re.Velocity.Y;
				robot2.velocity_z = re.Velocity.Z;
				robot2.touch = re.Touch;
				robot2.touch_normal_x = re.TouchNormal.X;
				robot2.touch_normal_y = re.TouchNormal.Y;
				robot2.touch_normal_z = re.TouchNormal.Z;
				auto robots2 = std::vector<model::Robot>();
				robots2.push_back(robot2);

				auto ball2 = model::Ball();
				ball2.x = be.Position.X;
				ball2.y = be.Position.Y;
				ball2.z = be.Position.Z;
				ball2.velocity_x = be.Velocity.X;
				ball2.velocity_y = be.Velocity.Y;
				ball2.velocity_z = be.Velocity.Z;
				ball2.radius = be.Radius;

				ms._ball = ball2;
				ms._robots = robots2;
				ms.InitBallEntities();		
				
				action = ms.SetAttackerAction(robot2, 1, bec, isOkBestBec, 0);
				if (abs(action.jump_speed - 15) < 1E-3) isJumping = true;
				re.Action = action;

				for (auto & bsp : ms._beforeStrikePoints)
				{
					ms._beforeStrikePoints[bsp.first].first--;
				}
			}

			Assert::AreEqual(counter, 1); //было 14 до новой системы атаки
			
			
		}

		TEST_METHOD(NeedJumpDefence)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = 17.538161397623792;
			ball.y = 9.361331651180034;
			ball.z = -37.36631178430045;
			ball.velocity_x = -13.760272642858897;
			ball.velocity_y = -15.300768836515168;
			ball.velocity_z = 31.82039045377821;
			ball.radius = Constants::Rules.BALL_RADIUS;


			model::Robot robot{};
			robot.x = 11.535674389380018;
			robot.y = 1;
			robot.z = -33.51693933513432;
			robot.velocity_x = 23.531036217874043;
			robot.velocity_y = 0;
			robot.velocity_z = 10.456605749849881;
			robot.radius = 1.0;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 1;
			robot.is_teammate = true;
			RobotEntity re = RobotEntity(robot);

			model::Robot oppRobot{};
			oppRobot.x = 11.663952816105612;
			oppRobot.y = 1;
			oppRobot.z = -15.80435525319085;
			oppRobot.velocity_x = 6.290134344718126;
			oppRobot.velocity_y = 0;
			oppRobot.velocity_z = -29.333158880785366;
			oppRobot.radius = 1.0;
			oppRobot.touch = true;
			oppRobot.touch_normal_x = 0;
			oppRobot.touch_normal_y = 1;
			oppRobot.touch_normal_z = 0;
			oppRobot.id = 4;
			oppRobot.is_teammate = false;

			auto robots = std::vector<model::Robot>();
			auto oppRobots = std::vector<model::Robot>();
			robots.push_back(robot);
			robots.push_back(oppRobot);
			oppRobots.push_back(oppRobot);


			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();
			ms._oppStrikeTime = ms.GetOppStrikeTime(oppRobots);

			std::optional<double> ct;
			BallEntity bestBe;
			bool isStrikeDefender;
			BallEntityContainer bec;
			bool isOkBestBec;
			bool isDribbling;
			int dribblingTicks;
			auto action = ms.SetAttackerAction(robot, 1, bec, isOkBestBec, 0);
			

			re.Action = action;
			auto be = BallEntity(ball);

			double deltaTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
			bool isGoalScored = false;

			const int aorCount = 4;
			model::Robot allOppRobots[aorCount];

			allOppRobots[0] = model::Robot();
			allOppRobots[0].x = 11.76878535993466;
			allOppRobots[0].y = 1;
			allOppRobots[0].z = -16.293241883969674;
			allOppRobots[0].velocity_x = 6.289952629740248;
			allOppRobots[0].velocity_y = 0;
			allOppRobots[0].velocity_z = -29.333197846733718;
			allOppRobots[0].radius = 1.0;
			allOppRobots[0].touch = true;
			allOppRobots[0].touch_normal_x = 0;
			allOppRobots[0].touch_normal_y = 1;
			allOppRobots[0].touch_normal_z = 0;
			allOppRobots[0].id = 4;
			allOppRobots[0].is_teammate = false;

			allOppRobots[1] = model::Robot();
			allOppRobots[1].x = 11.873614357679747;
			allOppRobots[1].y = 1;
			allOppRobots[1].z = -16.78212927512614;
			allOppRobots[1].velocity_x = 6.289739864701041;
			allOppRobots[1].velocity_y = 0;
			allOppRobots[1].velocity_z = -29.333243469387945;
			allOppRobots[1].radius = 1.0;
			allOppRobots[1].touch = true;
			allOppRobots[1].touch_normal_x = 0;
			allOppRobots[1].touch_normal_y = 1;
			allOppRobots[1].touch_normal_z = 0;
			allOppRobots[1].id = 4;
			allOppRobots[1].is_teammate = false;

			allOppRobots[2] = model::Robot();
			allOppRobots[2].x = 11.978439176570594;
			allOppRobots[2].y = 1;
			allOppRobots[2].z = -17.271017562308973;
			allOppRobots[2].velocity_x = 6.289489133453241;
			allOppRobots[2].velocity_y = 0;
			allOppRobots[2].velocity_z = -29.33329723096559;
			allOppRobots[2].radius = 1.0;
			allOppRobots[2].touch = true;
			allOppRobots[2].touch_normal_x = 0;
			allOppRobots[2].touch_normal_y = 1;
			allOppRobots[2].touch_normal_z = 0;
			allOppRobots[2].id = 4;
			allOppRobots[2].is_teammate = false;

			allOppRobots[3] = model::Robot();
			allOppRobots[3].x = 12.083259036381007;
			allOppRobots[3].y = 1;
			allOppRobots[3].z = -17.759906912765008;
			allOppRobots[3].velocity_x = 6.289191588626911;
			allOppRobots[3].velocity_y = 0;
			allOppRobots[3].velocity_z = -29.33336102736174;
			allOppRobots[3].radius = 1.0;
			allOppRobots[3].touch = true;
			allOppRobots[3].touch_normal_x = 0;
			allOppRobots[3].touch_normal_y = 1;
			allOppRobots[3].touch_normal_z = 0;
			allOppRobots[3].id = 4;
			allOppRobots[3].is_teammate = false;

			

			for (int k = 0; k < aorCount; ++k)
			{
				for (int i = 0; i < Constants::Rules.MICROTICKS_PER_TICK; ++i)
				{
					Simulator::Update(re, be, deltaTime / Constants::Rules.MICROTICKS_PER_TICK,
						(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
				}

				auto robot2 = model::Robot();
				robot2.x = re.Position.X;
				robot2.y = re.Position.Y;
				robot2.z = re.Position.Z;
				robot2.velocity_x = re.Velocity.X;
				robot2.velocity_y = re.Velocity.Y;
				robot2.velocity_z = re.Velocity.Z;
				robot2.touch = re.Touch;
				robot2.touch_normal_x = re.TouchNormal.X;
				robot2.touch_normal_y = re.TouchNormal.Y;
				robot2.touch_normal_z = re.TouchNormal.Z;


				auto robots2 = std::vector<model::Robot>();
				robots2.push_back(robot2);
				robots2.push_back(allOppRobots[k]);

				oppRobots = std::vector<model::Robot>();
				oppRobots.push_back(allOppRobots[k]);


				auto ball2 = model::Ball();
				ball2.x = be.Position.X;
				ball2.y = be.Position.Y;
				ball2.z = be.Position.Z;
				ball2.velocity_x = be.Velocity.X;
				ball2.velocity_y = be.Velocity.Y;
				ball2.velocity_z = be.Velocity.Z;
				ball2.radius = be.Radius;

				ms._ball = ball2;
				ms._robots = robots2;
				ms.InitBallEntities();
				ms._oppStrikeTime = ms.GetOppStrikeTime(oppRobots);

				action = ms.SetAttackerAction(robot2, 1, bec, isOkBestBec, 0);
				re.Action = action;
			}
		}

		TEST_METHOD(StrangeMoveToStrike)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -10.199910350729994;
			ball.y = 13.583038020123812;
			ball.z = 29.17185525114677;
			ball.velocity_x = 5.008919615878992;
			ball.velocity_y = -20.88831530721662;
			ball.velocity_z = 33.17536148270964;
			ball.radius = Constants::Rules.BALL_RADIUS;


			model::Robot robot{};
			robot.x = -10.329957812186107;
			robot.y = 1;
			robot.z = 30.20100649997507;
			robot.velocity_x = 2.7478458424436023;
			robot.velocity_y = 0;
			robot.velocity_z = 24.336465449703674;
			robot.radius = 1.0;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 1;
			robot.is_teammate = true;
			RobotEntity re = RobotEntity(robot);

			model::Robot oppRobot{};
			oppRobot.x = -4.602566358146782;
			oppRobot.y = 3.730006423392688;
			oppRobot.z = 40.53877132272065;
			oppRobot.velocity_x = -14.92669113596318;
			oppRobot.velocity_y = 7.996451780096161;
			oppRobot.velocity_z = -1.4665145680017055;
			oppRobot.radius = 1.05;
			oppRobot.touch = false;
			oppRobot.touch_normal_x = 0;
			oppRobot.touch_normal_y = 0;
			oppRobot.touch_normal_z = 0;
			oppRobot.id = 3;
			oppRobot.is_teammate = false;


			auto robots = std::vector<model::Robot>();
			auto oppRobots = std::vector<model::Robot>();
			robots.push_back(robot);
			robots.push_back(oppRobot);
			oppRobots.push_back(oppRobot);


			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();
			ms._oppStrikeTime = ms.GetOppStrikeTime(oppRobots);

			BallEntity bestBe;
			BallEntityContainer bec;
			bool isOkBestBec;
			bool isDribbling;
			int dribblingTicks;
			auto action = ms.SetAttackerAction(robot, 1, bec, isOkBestBec, 0);
		}


		TEST_METHOD(StrangeDefJump)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -0.6180202315380853;
			ball.y = 12.63312645516324;
			ball.z = -31.610930136253728;
			ball.velocity_x = 14.081836414308437;
			ball.velocity_y = -4.761414648981861;
			ball.velocity_z = 28.270220636045906;
			ball.radius = Constants::Rules.BALL_RADIUS;


			model::Robot robot{};
			robot.x = 4.582761738237277;
			robot.y = 1.5308088683817294;
			robot.z = -32.88346676137021;
			robot.velocity_x = 5.606167996043739;
			robot.velocity_y = 13.999249800454422;
			robot.velocity_z = 27.938115378030982;
			robot.radius = 1.05;
			robot.touch = false;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 0;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;

			model::Robot robot2{};
			robot2.x = -8.619403590463389;
			robot2.y = 1;
			robot2.z = -23.206724821763146;
			robot2.velocity_x = 28.943013021811247;
			robot2.velocity_y = 0;
			robot2.velocity_z = 7.893161421082464;
			robot2.radius = 1.0;
			robot2.touch = true;
			robot2.touch_normal_x = 0;
			robot2.touch_normal_y = 1;
			robot2.touch_normal_z = 0;
			robot2.id = 1;
			robot2.is_teammate = true;

			auto robots = std::vector<model::Robot>();
			auto oppRobots = std::vector<model::Robot>();
			robots.push_back(robot);
			robots.push_back(robot2);

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();
			ms._oppStrikeTime = 65.210664 / 60.0;

			
			BallEntityContainer bec;
			bool isOkBestBec;

			/*const auto collisionTick = int(c[robot.id].value() * Constants::Rules.TICKS_PER_SECOND);

			const model::Action robotAction = ms.SetAttackerAction(
				robot, collisionTick + 10,
				ms._beforeMyGates, bec, isOkBestBec);*/
		}

		TEST_METHOD(StrageStrikeJump)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -9.454973131489147;
			ball.y = 6.311844673647605;
			ball.z = 41.334087592196134;
			ball.velocity_x = 5.033299450348972;
			ball.velocity_y = 7.856756525515395;
			ball.velocity_z = -10.777785365857703;
			ball.radius = Constants::Rules.BALL_RADIUS;


			model::Robot robot{};
			robot.x = -7.965765388829436;
			robot.y = 1;
			robot.z = 22.667084713290393;
			robot.velocity_x = -0.7671446521661502;
			robot.velocity_y = 0;
			robot.velocity_z = 25.98002985731453;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;

			model::Robot oppRobot{};
			oppRobot.x = -8.792456365105082;
			oppRobot.y = 4.650838607350188;
			oppRobot.z = 44.37377022105135;
			oppRobot.velocity_x = -4.217082731814078;
			oppRobot.velocity_y = -13.214646825756619;
			oppRobot.velocity_z = 30.671440934225796;
			oppRobot.radius = 1.05;
			oppRobot.touch = false;
			oppRobot.touch_normal_x = 0;
			oppRobot.touch_normal_y = 0;
			oppRobot.touch_normal_z = 0;
			oppRobot.id = 3;
			oppRobot.is_teammate = false;

			auto robots = std::vector<model::Robot>();
			auto oppRobots = std::vector<model::Robot>();
			robots.push_back(robot);
			robots.push_back(oppRobot);

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			auto re = RobotEntity(robot);
			std::optional<double> jumpCollisionT = std::nullopt;
			std::optional<BallEntity> jump_ball_entity = std::nullopt;
			double changeDirVz = 0;
			double goalTime = 0;

			BallEntityContainer bec;
			bool isOkBestBec;

			bool isDribbling;
			int dribblingTicks;
			const model::Action robotAction = ms.SetAttackerAction(
				robot, 1,
				bec, isOkBestBec, 0);
			Assert::AreEqual(robotAction.jump_speed, 0, 1E-3);
		}

		TEST_METHOD(MaxJumpHeight)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -10.199910350729994;
			ball.y = 13.583038020123812;
			ball.z = 29.17185525114677;
			ball.velocity_x = 5.008919615878992;
			ball.velocity_y = -20.88831530721662;
			ball.velocity_z = 33.17536148270964;
			ball.radius = Constants::Rules.BALL_RADIUS;
			auto be = BallEntity(ball);


			model::Robot robot{};
			robot.x =0;
			robot.y = 1;
			robot.z = 0;
			robot.velocity_x = 0;
			robot.velocity_y = 0;
			robot.velocity_z = 0;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;

			model::Action action = model::Action();
			action.jump_speed = 15;

			auto re = RobotEntity(robot);
			re.Action = action;

			bool isGoalScoder;
			auto maxHeight = 0.0;
			auto maxHeightTime = 0.0;
			for (int i = 0; i < 100; ++i)
			{
				auto tickTime = 1.0 / 60;
				for (int j = 0; j < 100; ++j)
				{
					auto microTickTime = tickTime / 100;
					Simulator::Update(re, be, microTickTime, 0.45, isGoalScoder);
					if (re.Position.Y > maxHeight)
					{
						maxHeight = re.Position.Y;
						maxHeightTime = tickTime * i + microTickTime * j;
					}
				}
			}

		}

		TEST_METHOD(PvContainerTest)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -10.199910350729994;
			ball.y = 13.583038020123812;
			ball.z = 29.17185525114677;
			ball.velocity_x = 5.008919615878992;
			ball.velocity_y = -20.88831530721662;
			ball.velocity_z = 33.17536148270964;
			ball.radius = Constants::Rules.BALL_RADIUS;
			auto be = BallEntity(ball);


			model::Robot robot{};
			robot.x = 0;
			robot.y = 1;
			robot.z = 0;
			robot.velocity_x = -5;
			robot.velocity_y = 0;
			robot.velocity_z = -7;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;

			Vector3D targetPos = Vector3D(10, 1, 10);

			const auto robotPos = Helper::GetRobotPosition(robot);
			const auto robotVel = Helper::GetRobotVelocity(robot);

			int c = 40;
			auto pvConainer1 = Simulator::GetRobotPVContainer(
				robotPos,
				targetPos,
				robotVel,
				c,
				1
			);

			auto pvContainer2 = Simulator::GetRobotPVContainer(robotPos, targetPos, robotVel, 1, 1);
			for (int i = 0; i < c-1; ++i)
			{
				pvContainer2 = Simulator::GetRobotPVContainer(pvContainer2.Position, targetPos, pvContainer2.Velocity, 1, 1);
			}

			Assert::AreEqual(pvConainer1.Position.X, pvContainer2.Position.X, 1E-5);
			Assert::AreEqual(pvConainer1.Position.Y, pvContainer2.Position.Y, 1E-5);
			Assert::AreEqual(pvConainer1.Position.Z, pvContainer2.Position.Z, 1E-5);

			Assert::AreEqual(pvConainer1.Velocity.X, pvContainer2.Velocity.X, 1E-5);
			Assert::AreEqual(pvConainer1.Velocity.Y, pvContainer2.Velocity.Y, 1E-5);
			Assert::AreEqual(pvConainer1.Velocity.Z, pvContainer2.Velocity.Z, 1E-5);
		}

		
		TEST_METHOD(NeedScore2)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = -11.475231612076566;
			ball.y = 9.21156902559593;
			ball.z = 31.11715760598576;
			ball.velocity_x = 0.8029492548030697;
			ball.velocity_y = -9.400527464191002;
			ball.velocity_z = -5.080761505415911;
			ball.radius = Constants::Rules.BALL_RADIUS;
			auto be = BallEntity(ball);

			model::Robot robot{};
			robot.x = -16.37232589332971;
			robot.y = 1;
			robot.z = 3.581987585213731;
			robot.velocity_x = 5.811700624804918;
			robot.velocity_y = 0;
			robot.velocity_z = 9.714229038966918;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;

			auto robots = std::vector<model::Robot>();
			robots.push_back(robot);

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			bool isDspOk;
			BallEntityContainer bec;
			int bestMoveT;
			int bestWaitT;
			auto movePoint = ms.GetAttackerMovePoint(robot, 0, isDspOk, bec, bestWaitT, bestMoveT, 0);
			Assert::IsFalse(movePoint == std::nullopt);
		}

		TEST_METHOD(StopContainerTest)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);


			model::Robot robot{};
			robot.x = -16.37232589332971;
			robot.y = 1;
			robot.z = 3.581987585213731;
			robot.velocity_x = 5.811700624804918;
			robot.velocity_y = 0;
			robot.velocity_z = 9.714229038966918;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;

			double tickTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
			double microTickTime = tickTime / Constants::Rules.MICROTICKS_PER_TICK;

			/*const auto tv1Length = Constants::Rules.ROBOT_ACCELERATION * microTickTime;
			auto robotVelocity = Helper::GetRobotVelocity(robot);
			Vector3D tvc1 = Helper::GetTargetVelocity(robotVelocity, Vector3D(0, 0, 0), tv1Length);*/

			const auto robotPos = Helper::GetRobotPosition(robot);
			const auto robotVel = Helper::GetRobotVelocity(robot);
			auto stopContainer = ms.GetStopContainer(robotPos, robotVel);			

			auto waitT = 15;
			auto microTicks = waitT * Constants::Rules.MICROTICKS_PER_TICK;


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
				waitPos = stopContainer.stopPos;
				waitVelocity = Vector3D(0, 0, 0);
			}


			model::Ball ball{};
			ball.x = -11.475231612076566;
			ball.y = 9.21156902559593;
			ball.z = 31.11715760598576;
			ball.velocity_x = 0.8029492548030697;
			ball.velocity_y = -9.400527464191002;
			ball.velocity_z = -5.080761505415911;
			ball.radius = Constants::Rules.BALL_RADIUS;
			auto be = BallEntity(ball);
					   
			auto re = RobotEntity(robot);
			re.Action.target_velocity_x = 0;
			re.Action.target_velocity_y = 0;
			re.Action.target_velocity_z = 0;
			for (int i = 0; i < waitT; ++i)
				Simulator::Tick(re, be);

			Assert::AreEqual(re.Position.X, waitPos.X, 1E-5);
			Assert::AreEqual(re.Position.Y, waitPos.Y, 1E-5);
			Assert::AreEqual(re.Position.Z, waitPos.Z, 1E-5);


			Assert::AreEqual(re.Velocity.X, waitVelocity.X, 1E-5);
			Assert::AreEqual(re.Velocity.Y, waitVelocity.Y, 1E-5);
			Assert::AreEqual(re.Velocity.Z, waitVelocity.Z, 1E-5);

		}

		TEST_METHOD(NeedDefend)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = 14.583370657468574;
			ball.y = 3.4800643262723545;
			ball.z = -26.727762817119135;
			ball.velocity_x = -10.621424469614489;
			ball.velocity_y = 13.576501623107495;
			ball.velocity_z = -31.25600752212418;
			ball.radius = Constants::Rules.BALL_RADIUS;
			


			model::Robot robot{};
			robot.x = 10.7963495388937;
			robot.y = 1;
			robot.z = -31.107585814970037;
			robot.velocity_x = 3.6566930039505507e-13;
			robot.velocity_y = 0;
			robot.velocity_z = 3.3366538698675896e-13;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;

			auto robots = std::vector<model::Robot>();
			robots.push_back(robot);

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			BallEntityContainer bec;
			bool isOk;
			bool isDribbling;
			int dribblingTicks;
			auto action = ms.SetAttackerAction(robot, 0, bec, isOk, 0);
			Assert::AreEqual(std::get<1>(ms._defenderMovePoints[robot.id]), 0);
			Assert::AreEqual(std::get<2>(ms._defenderMovePoints[robot.id]), 2);

			auto be = BallEntity(ball);
			auto re = RobotEntity(robot);
			re.Action = action;

			double deltaTime = 1.0 / Constants::Rules.TICKS_PER_SECOND;
			bool isGoalScored = false;
			for (int i = 0; i < Constants::Rules.MICROTICKS_PER_TICK; ++i)
			{
				Simulator::Update(re, be, deltaTime / Constants::Rules.MICROTICKS_PER_TICK,
					(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
			}



			ball.x = be.Position.X;
			ball.y = be.Position.Y;
			ball.z = be.Position.Z;
			ball.velocity_x = be.Velocity.X;
			ball.velocity_y = be.Velocity.Y;
			ball.velocity_z = be.Velocity.Z;
			ball.radius = Constants::Rules.BALL_RADIUS;

			robot.x = re.Position.X;
			robot.y = re.Position.Y;
			robot.z = re.Position.Z;
			robot.velocity_x = re.Velocity.X;
			robot.velocity_y = re.Velocity.Y;
			robot.velocity_z = re.Velocity.Z;

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			action = ms.SetAttackerAction(robot, 0, bec, isOk, 0);
			Assert::AreEqual(std::get<1>(ms._defenderMovePoints[robot.id]), 0);
			Assert::AreEqual(std::get<2>(ms._defenderMovePoints[robot.id]), 1);

			be = BallEntity(ball);
			re = RobotEntity(robot);
			re.Action = action;


			for (int i = 0; i < Constants::Rules.MICROTICKS_PER_TICK; ++i)
			{
				Simulator::Update(re, be, deltaTime / Constants::Rules.MICROTICKS_PER_TICK,
					(Constants::Rules.MIN_HIT_E + Constants::Rules.MAX_HIT_E) / 2.0, isGoalScored);
			}

			ball.x = be.Position.X;
			ball.y = be.Position.Y;
			ball.z = be.Position.Z;
			ball.velocity_x = be.Velocity.X;
			ball.velocity_y = be.Velocity.Y;
			ball.velocity_z = be.Velocity.Z;
			ball.radius = Constants::Rules.BALL_RADIUS;

			robot.x = re.Position.X;
			robot.y = re.Position.Y;
			robot.z = re.Position.Z;
			robot.velocity_x = re.Velocity.X;
			robot.velocity_y = re.Velocity.Y;
			robot.velocity_z = re.Velocity.Z;

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			action = ms.SetAttackerAction(robot, 0, bec, isOk, 0);
			Assert::AreEqual(std::get<1>(ms._defenderMovePoints[robot.id]), 0);
			Assert::AreEqual(std::get<2>(ms._defenderMovePoints[robot.id]), 0);

		}

		TEST_METHOD(StrageDefence)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			model::Ball ball{};
			ball.x = 4.02800758694681;
			ball.y = 9.225490651334818;
			ball.z = -23.52447773844161;
			ball.velocity_x = 21.047978698976856;
			ball.velocity_y = -19.28871470838649;
			ball.velocity_z = -30.12103980936056;
			ball.radius = Constants::Rules.BALL_RADIUS;



			model::Robot robot{};
			robot.x = 3.8165676195389264;
			robot.y = 1;
			robot.z = -35.128984938229316;
			robot.velocity_x = 16.35087876988658;
			robot.velocity_y = 0;
			robot.velocity_z = 25.152510082543895;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;

			model::Robot oppRobot{};
			oppRobot.x = 6.816472999713154;
			oppRobot.y = 4.647359026809699;
			oppRobot.z = -22.190822943754522;
			oppRobot.velocity_x = 6.050013816840022;
			oppRobot.velocity_y = 2.9996463780140923;
			oppRobot.velocity_z = -29.35920325518888;
			oppRobot.radius = 1.05;
			oppRobot.touch = false;
			oppRobot.touch_normal_x = 0;
			oppRobot.touch_normal_y = 0;
			oppRobot.touch_normal_z = 0;
			oppRobot.id = 3;
			oppRobot.is_teammate = false;

			auto robots = std::vector<model::Robot>();
			auto oppRobots = std::vector<model::Robot>();
			robots.push_back(robot);
			robots.push_back(oppRobot);

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			BallEntityContainer bec;
			bool isOk;
			bool isDribbling;
			int dribblingTicks;
			auto action = ms.SetAttackerAction(robot, 0, bec, isOk, 0);
		}

		TEST_METHOD(isGoalDirection)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);

			const auto pos = Vector3D(-20, 2, -20);
			const auto vel = Vector3D(-5, 0, -10);
			/*const auto isGoal = ms.IsOppGatesDirection(pos, vel);
			Assert::IsFalse(isGoal);*/
		}

		TEST_METHOD(Nitro)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);


			model::Ball ball{};
			ball.x = 4.02800758694681;
			ball.y = 9.225490651334818;
			ball.z = -23.52447773844161;
			ball.velocity_x = 21.047978698976856;
			ball.velocity_y = -19.28871470838649;
			ball.velocity_z = -30.12103980936056;
			ball.radius = Constants::Rules.BALL_RADIUS;



			model::Robot robot{};
			robot.x = 3.8165676195389264;
			robot.y = 1;
			robot.z = -35.128984938229316;
			robot.velocity_x = 25;
			robot.velocity_y = 0;
			robot.velocity_z = 0;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;
			robot.nitro_amount = 100;

			model::Action action = model::Action();
			action.target_velocity_x = 25;
			action.target_velocity_y = 0;
			action.target_velocity_z = 0;
			action.use_nitro = false;
			action.jump_speed = Constants::Rules.ROBOT_MAX_JUMP_SPEED;

			auto re = RobotEntity(robot);
			re.Action = action;

			auto be = BallEntity(ball);

			bool isGoalScored;
			const auto time = 1.0 / 60 / 100;

			auto hs = std::vector<double>();
			auto vys = std::vector<double>();
			auto vxs = std::vector<double>();
			auto vzs = std::vector<double>();
			auto nitros = std::vector<double>();

			for (int i = 0; i < 60; ++i)
			{
				if (i == 3)
				{
					re.Action.use_nitro = true;
					re.Action.target_velocity_x = 25;
					re.Action.target_velocity_y = 100;
				}

				for (int j = 0; j < 100; ++j)
				{
					Simulator::Update(re, be, time, 0.45, isGoalScored);
					hs.push_back(re.Position.Y);
					vxs.push_back(re.Velocity.X);
					vys.push_back(re.Velocity.Y);
					vzs.push_back(re.Velocity.Z);
					nitros.push_back(re.Nitro);
				}

				
			}
		}

		TEST_METHOD(NitroJump)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);


			model::Ball ball{};
			ball.x = -6.531225742;
			ball.y = 15.997321679123;
			ball.z = 6.41798355512397;
			ball.velocity_x = -3.82217519187906;
			ball.velocity_y = 20.1611388402691;
			ball.velocity_z = 32.6691849838708;
			ball.radius = Constants::Rules.BALL_RADIUS;



			model::Robot robot{};
			robot.x = -15.5011064785692;
			robot.y = 1;
			robot.z = 6.70051829622676;
			robot.velocity_x = 13.4710818135258;
			robot.velocity_y = 0;
			robot.velocity_z = 26.8054090581228;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;
			robot.nitro_amount = 100;


			auto robots = std::vector<model::Robot>();
			robots.push_back(robot);

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			BallEntityContainer bec;
			bool isOk;
			auto action = ms.SetAttackerAction(robot, 0, bec, isOk, 0);
			Assert::IsFalse(action.use_nitro);
		}

		TEST_METHOD(NitroJump2)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);


			model::Ball ball{};
			ball.x = -15.42178764530815;
			ball.y = 9.213765787804567;
			ball.z = -6.028610796177337;
			ball.velocity_x = 2.693440570395559;
			ball.velocity_y = 17.713517867123965;
			ball.velocity_z = 34.29372230337883;
			ball.radius = Constants::Rules.BALL_RADIUS;



			model::Robot robot{};
			robot.x = 3.941760965136863;
			robot.y = 1;
			robot.z = 37.00409267735491;
			robot.velocity_x = -19.41297112187636;
			robot.velocity_y = 0;
			robot.velocity_z = -13.457801771705302;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;
			robot.nitro_amount = 100;


			auto robots = std::vector<model::Robot>();
			robots.push_back(robot);

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			BallEntityContainer bec;
			bool isOk;
			auto action = ms.SetAttackerAction(robot, 0, bec, isOk, 0);
			Assert::IsFalse(action.use_nitro);
		}

		TEST_METHOD(WrongAttPoint)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);


			model::Ball ball{};
			ball.x = -0.25541651386734887;
			ball.y = 2.3621735404335524;
			ball.z = -3.532802343855206;
			ball.velocity_x = 16.035388737162275;
			ball.velocity_y = 7.673328515712813;
			ball.velocity_z = 14.13089052269252;
			ball.radius = Constants::Rules.BALL_RADIUS;



			model::Robot robot{};
			robot.x = 0.22291522035105651;
			robot.y = 1;
			robot.z = -8.255653412943966;
			robot.velocity_x = 18.11204336772436;
			robot.velocity_y = 0;
			robot.velocity_z = 23.91555738521835;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;
			robot.nitro_amount = 100;


			auto robots = std::vector<model::Robot>();
			robots.push_back(robot);

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();

			BallEntityContainer bec;
			bool isOk;
			auto action = ms.SetAttackerAction(robot, 0, bec, isOk, 0);
		}

		TEST_METHOD(NeedDefenceNitro)
		{
			model::Rules rules = get_rules();
			MyStrategy ms{};
			Constants::Rules = rules;
			ms.Init(rules);
			DanCalculator::Init(rules.arena);
			

			model::Ball ball{};
			ball.x = -3.09919676692765;
			ball.y = 11.2542370645916;
			ball.z = -37.377634241938;
			ball.velocity_x = 6.17857333012381;
			ball.velocity_y = -9.07938768303758;
			ball.velocity_z = 13.3364091013326;
			ball.radius = Constants::Rules.BALL_RADIUS;



			model::Robot robot{};
			robot.x = -3.275826304432E-19;
			robot.y = 1;
			robot.z = -40.9999999999999;
			robot.velocity_x = -3.02390762169785E-19;
			robot.velocity_y = 0;
			robot.velocity_z = -1.06581410364015E-12;
			robot.radius = 1;
			robot.touch = true;
			robot.touch_normal_x = 0;
			robot.touch_normal_y = 1;
			robot.touch_normal_z = 0;
			robot.id = 2;
			robot.is_teammate = true;
			robot.nitro_amount = 5.83333333333561;


			auto robots = std::vector<model::Robot>();
			robots.push_back(robot);

			ms._ball = ball;
			ms._robots = robots;
			ms.InitBallEntities();
			ms._meGoalScoringTick = 42;

			BallEntityContainer bec;
			bool isOk;
			auto action = ms.SetAttackerAction(robot, 0, bec, isOk, -1);
		}

	};
};