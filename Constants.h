#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include "model/Rules.h"

struct Constants
{
public:
	static model::Rules Rules;
};

#endif