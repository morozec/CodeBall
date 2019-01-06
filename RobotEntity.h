#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _ROBOTENTITY_H_
#define _ROBOTENTITY_H_

#include "Vector3D.h"
#include "Entity.h"
#include "Constants.h"
#include "model/Action.h"
#include "model/Robot.h"

struct RobotEntity :Entity {
	RobotEntity(const model::Robot& robot);
	RobotEntity(const RobotEntity& robotEntity);
	RobotEntity(const Vector3D& position, const Vector3D& velocity, double radius, bool touch,
		const Vector3D touchNormal, double nitro);

	virtual double GetMass() override;
	virtual double GetRadiusChangeSpeed() const override;
	virtual void SetRadiusChangeSpeed(double value) override;
	virtual double GetArenaE() override;

	bool Touch;
	Vector3D TouchNormal;
	double Nitro;
	model::Action Action;
};

#endif