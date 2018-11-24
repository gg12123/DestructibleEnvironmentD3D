#pragma once
#include "CollisionData.h"

class Rigidbody;

class SplitInfo
{
public:
	SplitInfo(Rigidbody& toSplit, const Impulse& cause) : CauseImpulse(cause)
	{
		ToSplit = &toSplit;
	}

	Rigidbody * ToSplit;
	Impulse CauseImpulse;
};
