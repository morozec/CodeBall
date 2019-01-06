#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _POSITIONVELOCITYCONTAINER_H_
#define _POSITIONVELOCITYCONTAINER_H_

#include "Vector3D.h"

struct PositionVelocityContainer
{
	Vector3D Position;
	Vector3D Velocity;
	bool IsPassedBy;

	PositionVelocityContainer(Vector3D& position, Vector3D& velocity, bool isPassedBy);
};

#endif