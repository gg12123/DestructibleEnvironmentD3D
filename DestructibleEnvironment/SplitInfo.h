#pragma once

class Rigidbody;
class Impulse;

class SplitInfo
{
public:
	SplitInfo(Rigidbody& toSplit, Impulse& cause)
	{
		ToSplit = &toSplit;
		CauseImpulse = &cause;
	}

	Rigidbody * ToSplit;
	Impulse* CauseImpulse;
};
