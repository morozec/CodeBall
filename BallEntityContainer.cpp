#include "BallEntityContainer.h"

BallEntityContainer::BallEntityContainer(
	BallEntity ballEntity, double collisionTime, bool isGoalScored, double goalTime, int collisionsCount, BallEntity collideBallEntity, int nitroDest)
{
	this->ResBallEntity = ballEntity;
	this->collisionTime = collisionTime;
	this->isGoalScored = isGoalScored;
	this->goalTime = goalTime;
	this->collisionsCount = collisionsCount;
	this->CollideBallEntity = collideBallEntity;
	this->NitroDest = nitroDest;
}

double BallEntityContainer::GetFullGoalTime() const
{
	return collisionTime + goalTime;
}
