#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _BALLENTITY_H_
#define _BALLENTITY_H_

#include "Vector3D.h"
#include "Entity.h"
#include "Constants.h"
#include "model/Ball.h"

struct BallEntity :Entity {
	BallEntity() = default; //TODO:������
	BallEntity(const model::Ball& ball);
	BallEntity(const BallEntity& ballEntity);

	virtual double GetMass() override;
	virtual double GetRadiusChangeSpeed() const override;
	virtual void SetRadiusChangeSpeed(double value) override;
	virtual double GetArenaE() override;
};

#endif