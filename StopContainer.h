#pragma once
#include "Vector3D.h"

struct StopContainer
{
	Vector3D robotPosition;
	Vector3D robotVelocity;

	int stopMicroTicks;
	Vector3D stopPos;
	Vector3D stopA;


	StopContainer(const Vector3D& robot_position, const Vector3D& robot_velocity, int stop_micro_ticks,
		const Vector3D& stop_pos, const Vector3D& stop_a)
		: robotPosition(robot_position),
		  robotVelocity(robot_velocity),
		  stopMicroTicks(stop_micro_ticks),
		  stopPos(stop_pos),
		  stopA(stop_a)
	{
	}
};
