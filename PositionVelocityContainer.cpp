#include "PositionVelocityContainer.h"

PositionVelocityContainer::PositionVelocityContainer(Vector3D& position, Vector3D& velocity, bool isPassedBy)
{
	Position = position;
	Velocity = velocity;
	IsPassedBy = isPassedBy;
}
