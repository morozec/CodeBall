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
	const int MaxCollisionsToGoalStrike = 3;
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

	
	double _distToFollowBall;
	
	
	

	//int _defenderId = -1;

	bool _isFirstRobot = false;
	std::map<int, model::Action> _actions;
	
	
	

	std::vector<Sphere> _drawSpheres;

	bool _isGoalPossible;
	bool _isMeGoalPossible;
	bool _isNoCollisionGoalPossible;
	bool _isNoCollisionMeGoalPossible;
	
public:
	int _goalScoringTick;
	Vector3D _myGates;
	Vector3D _beforeMyGates;
	std::optional<double> _oppStrikeTime;

	double _hitEs[2];
	double _allHitEs[3];
	double _averageHitE[1];
	model::Ball _ball;
	std::vector<model::Robot> _robots;

	std::map<int, BallEntity> _ballEntities;
	std::map<int, std::vector<RobotEntity>> _robotEntities;

	std::map<int, std::pair<int, Vector3D>> _beforeStrikePoints;
	std::map<int, std::pair<int, int>> _defenderMovePoints; //id ������: t ����, t ������


	void Init(const model::Rules& rules);

	void InitAction(model::Action& action, int id);

	void InitJumpingRobotAction(const model::Robot& robot, const model::Ball& ball);
	//std::vector<RobotEntity> GetJumpRobotEntities();

	model::Action GetDefaultAction(const model::Robot& me, const Vector3D& defaultPos);	
	BallEntity SimulateTickBall(
		const BallEntity& ballEntity, std::vector<RobotEntity>& jumpRes, bool& isGoalScored, bool discardIsCollided) const;
	bool SimulateCollision(BallEntity& ballEntity, RobotEntity& robotEntity,
	                       const double hitEs[], int hitEsSize,
		std::vector<BallEntity>& resBes, std::vector<double>& resCollisionTimes,
		int beforeTicks, bool forbidNegativeVy, bool forbidDownStrike);

	bool SimulateFullCollision(BallEntity & be, std::vector<RobotEntity>& res, double& collisionT, double hitE, bool forbidNegativeVy, bool forbidDownStrike) const;

	bool IsPenaltyArea(const Vector3D& position, bool isDefender) const;
	double GetVectorAngleToHorizontal(const Vector3D& v) const;
	int CompareDefenderBallEntities(const BallEntity & b1, const std::optional<BallEntity>& b2, int& isB2GoalDirection) const;
	bool IsGoalBallDirection2(const BallEntity& startBallEntity, int directionCoeff, bool considerBoardSide, double& goalTime) const;

	model::Action SetDefenderAction(const model::Robot& me,
		const Vector3D& defenderPoint,
		std::optional<double>& collisionT, BallEntity& bestBallEntity);
	std::optional<BallEntity> GetDefenderStrikeBallEntity(
		const model::Robot& robot, int t,
		int startAttackTick,
		int endAttackTick,
		bool isDefender,
		std::optional<double>& collisionT, bool& isPassedBy, int& bestMoveT);
	bool IsOkDefenderPosToJump(
		const Vector3D & robotPosition, const Vector3D & robotVelocity,
		bool isDefender,
		int beforeTicks,
		std::optional<double>& jumpCollisionT, std::optional<BallEntity>& collisionBallEntity);
	std::optional<Vector3D> GetDefenderMovePoint(const model::Robot& robot,
		std::optional<double>& collisionT, BallEntity& bestBallEntity, bool& isSavedPointOk);

	model::Action SetAttackerAction(const model::Robot& me, 
		int startAttackTick,
		const Vector3D& defenderPoint,
		std::optional<double>& collisionT, BallEntity& bestBallEntity, bool& isDefender, bool& isStrikeDefender);
	bool IsOkPosToMove(const Vector3D& mePos, const model::Robot& robot, int t,
		int directionCoeff,
		std::optional<double>& collisionT, std::optional<BallEntity>& bestBallEntity, double& goalTime);
	std::optional<Vector3D> GetAttackerMovePoint(const model::Robot& robot, 
		int startAttackTick,
		std::optional<double>& collisionT, bool& isDefender, bool& isStrikeDefender, BallEntity& bestBallEntity, bool& isSavedPointOk, double& bestGoalTime);
	std::optional<Vector3D> GetAttackerStrikePoint(
		const model::Robot& robot, int t, int directionCoeff, 
		std::optional<double>& collisionT, std::optional<BallEntity>& bestBallEntity, double& goalTime);
	bool IsOkPosToJump(
		RobotEntity& robotEntity,
		int beforeTicks,
		std::optional<double>& collisionT, std::optional<BallEntity>& bestBallEntity, double& goalTime);

	bool IsOkOppPosToJump(
		RobotEntity& robotEntity,
		int beforeTicks,
		std::optional<double>& collisionT);

	static Vector3D GetDefendPointTargetVelocity(const model::Robot& robot, const Vector3D& position);
	std::optional<double> GetOppStrikeTime(const std::vector<model::Robot>& oppRobots);
	static model::Robot get_nearest_ball_robot(const BallEntity& ball_entity, const std::vector<model::Robot>& oppRobots);

	void InitBallEntities(
		std::map<int, std::optional<double>>& collisionTimes,
		std::map<int, BallEntity>& bestBallEntities);
	int UpdateBallEntities(double collisionTime, const Vector3D& afterCollisionBallVelocity);


	

    MyStrategy();
    void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;
	std::string custom_rendering() override;
};

#endif
