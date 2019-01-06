#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _ENTITY_H_
#define _ENTITY_H_

#include "Vector3D.h"

struct Entity{
protected:
	double _radiusChangeSpeed = 0;

public:
	virtual ~Entity() = default;
	Vector3D Position;
	Vector3D Velocity;
	double Radius;
	virtual double GetMass() = 0;	
	virtual double GetRadiusChangeSpeed() const = 0;
	virtual void SetRadiusChangeSpeed(double value) = 0;
	virtual double GetArenaE() = 0;

	bool IsCollided;
	bool IsArenaCollided;
};


#endif
