#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _DAN_H_
#define _DAN_H_

#include "Vector3D.h"

struct Dan {
	double Distance;
	Vector3D Normal;

	Dan() {};
	Dan(double distance, const Vector3D& normal);
};

#endif