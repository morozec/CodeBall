#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif
#include <optional>

#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include "Entity.h"
#include "DanCalculator.h"	
#include "PositionVelocityContainer.h"	
#include "RobotEntity.h"	
#include "BallEntity.h"	

class Simulator {
private:
	static const double Eps;
	static const double Eps2;	
	
	static double GetRobotRadius(double jumpSpeed);

	static void CollideEntities(Entity & a, Entity & b, double hitE);
	static std::optional<Vector3D> CollideWithArena(Entity & e);
	
public:
	static PositionVelocityContainer GetRobotPVContainer(const Vector3D& startPosition, 
		const Vector3D& targetPosition,
		const Vector3D& startVelocity, int ticks, double velocityCoeff);	
	static void Move(Entity & e, double deltaTime);
	static std::optional<double> GetCollisionT(const Vector3D& pR, const Vector3D& vR, const Vector3D& pB, const Vector3D& vB);
	static void Update(Entity& entity, double deltaTime, bool& isGoalScored);
	static void Update(RobotEntity& robot, BallEntity& ball, double deltaTime, double hitE, bool& isGoalScored);
	static void Tick(Entity& ball);
	static void Tick(RobotEntity& robot, BallEntity ball);
};

#endif