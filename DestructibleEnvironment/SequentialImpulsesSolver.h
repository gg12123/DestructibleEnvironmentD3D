#pragma once
#include "ContactPoints.h"
#include "CollisionResponder.h"

class SequentialImpulsesSolver
{
private:
	void ApplyImpulseAt(ContactPoint& p)
	{
		auto prevAccImpulse = p.GetAccumulatedImpulse();

		auto delta = p.CalculateImpulse();
		p.SetAccumulatedImpulse(MathU::Max(prevAccImpulse + delta, 0.0f));

		auto change = p.GetAccumulatedImpulse() - prevAccImpulse;
		p.ApplyImpulse(change);
	}

public:
	void Solve(std::vector<ContactPoint>& contactPoints)
	{
		for (auto i = 0; i < 10; i++)
		{
			for (auto& c : contactPoints)
				ApplyImpulseAt(c);
		}
	}
};
