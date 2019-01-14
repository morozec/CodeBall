#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "Sphere.h"


#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include <map>
#include "Constants.h"
#include "Vector3D.h"
#include "BallEntity.h"
#include "RobotEntity.h"
#include <optional>

class MyStrategy : public Strategy {

private:
	const double EPS = 1E-5;
	const int BallMoveTicks = 100;
	const int StrikeSphereStepsCount = 10;
	const int AttackerAddTicks = 10;
	double _maxStrikeDist;
	double _maxStrikeDist2;

	double _maxDefenderStrikeDist;		
	double _maxDefenderStrikeDist2;

	double _moveSphereRadius;
		
	double _penaltyAreaZ;
	double _penaltyMinX;
	double _penaltyMaxX;

	Vector3D _oppGates;
	Vector3D _oppGatesLeftCorner;
	Vector3D _oppGatesRightCorner;
	Vector3D _myGatesLeftCorner;
	Vector3D _myGatesRightCorner;

	Vector3D _myGates;
	Vector3D _beforeMyGates;
	double _distToFollowBall;
	double _hitEs[2];
	
	std::map<int, Vector3D> _beforeStrikePoints;

	//int _defenderId = -1;

	bool _isFirstRobot = false;
	std::map<int, model::Action> _actions;
	std::map<int, BallEntity> _ballEntities;
	std::map<int, std::vector<RobotEntity>> _robotEntities;
	std::optional<double> _oppStrikeTime;
	int _goalScoringTick;

	std::vector<Sphere> _drawSpheres;
	model::Ball _ball;
	std::vector<model::Robot> _robots;
public:
	void Init(const model::Rules& rules);

	void InitAction(model::Action& action, int id);

	void InitJumpingRobotAction(const model::Robot& robot, const model::Ball& ball);
	//std::vector<RobotEntity> GetJumpRobotEntities();

	model::Action GetDefaultAction(const model::Robot& me, const Vector3D& defaultPos);	
	BallEntity SimulateTickBall(
		const BallEntity& ballEntity, std::vector<RobotEntity>& jumpRes, bool& isGoalScored, bool discardIsCollided) const;
	bool SimulateCollision(BallEntity& ballEntity, RobotEntity& robotEntity, 
		std::optional<double>& collisionT, std::vector<RobotEntity>& jumpingRes, int beforeTicks);
	bool SimulateNoTouchEntitiesCollision(
		BallEntity& ballEntity, RobotEntity& robotEntity, double robotCollisionT, double ballCollisionT, int beforeTicks);

	bool SimulateFullCollision(BallEntity & be, std::vector<RobotEntity>& res, double& collisionT) const;

	bool IsPenaltyArea(const Vector3D& position, bool isDefender) const;
	double GetVectorAngleToHorizontal(const Vector3D& v) const;
	int CompareBallVelocities(const Vector3D& v1, const std::optional<Vector3D>& v2);
	bool IsGoalBallDirection2(const BallEntity& startBallEntity, int directionCoeff, double ballEntityTime);

	model::Action SetDefenderAction(const model::Robot& me,
		const Vector3D& defenderPoint, bool isMeGoalPossible, 
		std::optional<double>& collisionT, Vector3D& bestBallVelocity);
	std::optional<Vector3D> GetDefenderStrikeBallVelocity(
		const model::Robot& robot, int t,
		int startAttackTick,
		bool isMeGoalPossible,
		bool isDefender,
		std::optional<double>& collisionT, bool& isPassedBy);
	bool IsOkDefenderPosToJump(
		const Vector3D & robotPosition, const Vector3D & robotVelocity,
		bool isMeGoalPossible,
		bool isDefender,
		int beforeTicks,
		std::optional<double>& jumpCollisionT, std::optional<Vector3D>& collisionBallVelocity);
	std::optional<Vector3D> GetDefenderMovePoint(const model::Robot& robot,
		bool isMeGoalPossible,
		std::optional<double>& collisionT, Vector3D& bestBallVelocity);

	model::Action SetAttackerAction(const model::Robot& me, 
		bool isMeGoalPossible,
		int startAttackTick,
		const Vector3D& defenderPoint,
		std::optional<double>& collisionT, Vector3D& bestBallVelocity, bool& isDefender);
	bool IsOkPosToMove(const Vector3D& mePos, const model::Robot& robot, int t,
		int directionCoeff,
		std::optional<double>& collisionT, std::optional<Vector3D>& bestBallVelocity);
	std::optional<Vector3D> GetAttackerMovePoint(const model::Robot& robot, 
		bool isMeGoalPossible,
		int startAttackTick,
		std::optional<double>& collisionT, bool& isDefender, Vector3D& bestBallVelocity);
	std::optional<Vector3D> GetAttackerStrikePoint(
		const model::Robot& robot, int t, int directionCoeff, 
		std::optional<double>& collisionT, std::optional<Vector3D>& bestBallVelocity);
	bool IsOkPosToJump(
		RobotEntity& robotEntity,
		int beforeTicks,
		std::optional<double>& collisionT, std::optional<Vector3D>& bestBallVelocity);

	bool IsOkOppPosToJump(
		RobotEntity& robotEntity,
		int beforeTicks,
		std::optional<double>& collisionT);

	static Vector3D GetDefendPointTargetVelocity(const model::Robot& robot, const Vector3D& position);
	std::optional<double> GetOppStrikeTime(const std::vector<model::Robot>& oppRobots);
	static model::Robot get_nearest_ball_robot(const BallEntity& ball_entity, const std::vector<model::Robot>& oppRobots);

	void InitBallEntities(const model::Ball& ball, const std::vector<model::Robot>& robots,
		std::map<int, std::optional<double>>& collisionTimes,
		std::map<int, Vector3D>& bestBallVelocities);
	int UpdateBallEntities(double collisionTime, const Vector3D& afterCollisionBallVelocity);


	

    MyStrategy();
    void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;
	std::string custom_rendering() override;
};

#endif
