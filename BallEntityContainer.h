#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif
#include "BallEntity.h"

#ifndef _BallEntityContainer_H_
#define _BallEntityContainer_H_

class BallEntityContainer
{
public:
	BallEntity ResBallEntity;
	double collisionTime;//время на разгон и прыжок
	bool isGoalScored;
	double goalTime;//время после прыжка до гола
	int collisionsCount;
	BallEntity CollideBallEntity;//мяч после коллизии с ареной

	int NitroDest; //-1 - на работа врага, 0 - без нитры, 1 - чтобы забить гол

	BallEntityContainer()
	{
		ResBallEntity = BallEntity(); collisionTime = -1; 
		isGoalScored = false; goalTime = -1; collisionsCount = 0;
		CollideBallEntity = BallEntity(); NitroDest = 0;
	}
	BallEntityContainer(
		BallEntity ballEntity, double collisionTime, bool isGoalScored, double goalTime,
		int collisionsCount, BallEntity collideBallEntity, int nitroDest);

	double GetFullGoalTime() const;
};

#endif

