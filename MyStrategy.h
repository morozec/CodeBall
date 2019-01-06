#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include <cmath>
#include <algorithm>
#include <map>
#include <vector>
#include "Constants.h"
#include "Vector3D.h"
#include "BallEntity.h"
#include "RobotEntity.h"
#include "DanCalculator.h"
#include "Simulator.h"

class MyStrategy : public Strategy {
public:
private:
	const double EPS = 1E-5;
	const double DeltaAngleToStrike = M_PI / 180 * 3;
	const int BallMoveTicks = 100;
	const int CollideWaitTicks = 50;
	const int StrikeSphereStepsCount = 10;
	double _maxStrikeDist;
	double _maxStrikeDist2;

	double _maxDefenderStrikeDist;		
	double _maxDefenderStrikeDist2;

	double _moveSpereRadius;
		
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
	
	std::map<int, Vector3D> _beforeStrikePoints;

	int _defenderId = -1;

	bool _isFirstRobot = false;
	std::map<int, model::Action> _actions;
	std::map<int, BallEntity> _ballEntities;

	void Init(const model::Rules& rules);

	void InitAction(model::Action& action, int id);
	model::Action GetDefaultAction(const model::Robot& me, const Vector3D& defaultPos);	
	BallEntity SimulateTickBall(const BallEntity& ballEntity, bool& isGoalScored);
	bool IsPenaltyArea(const Vector3D& position);
	double GetVectorAngleToHorizontal(const Vector3D& v);
	int CompareBallVelocities(const Vector3D& v1, const Vector3D* v2);
	bool IsGoalBallDirection2(const BallEntity& ballEntity, int directionCoeff);

	model::Action SetDefenderAction(const model::Robot& me, const model::Ball& ball, const Vector3D& defenderPoint, double*& collisionT);
	Vector3D* GetDefenderStrikeBallVelocity(
		const model::Robot& robot, int t, double*& collisionT, bool& isPassedBy);
	bool IsOkDefenderPosToJump(const Vector3D & robotPosition, const Vector3D & robotVelocity,
		const Vector3D& moveTBePosition, const Vector3D& moveTBeVelocity,
		double*& collisionT, Vector3D*& collisionBallVelocity);
	Vector3D* GetDefenderMovePoint(const model::Robot& robot, const model::Ball& ball, double*& collisionT, Vector3D*& bestBallVelocity);

	model::Action SetAttackerAction(const model::Robot& me, const model::Ball& ball, double*& collisionT);
	bool IsOkPosToMove(const Vector3D& mePos, const model::Robot& robot, const BallEntity& ballEntity, int t, double*& collisionT);
	Vector3D* GetAttackerMovePoint(const model::Robot& robot, const model::Ball& ball,
		double*& collisionT, bool& isDefender, Vector3D*& bestBallVelocity);
	Vector3D* GetAttackerStrikePoint(const model::Robot& robot, int t, double*& collisionT);
	bool IsOkPosToJump(
		BallEntity& ballEntity,
		RobotEntity& robotEntity,
		double*& collisionT);


public:
    MyStrategy();
    void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;
};

#endif
