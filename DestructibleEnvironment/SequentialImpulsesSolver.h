#pragma once
#include "ContactPoints.h"
#include "CollisionResponder.h"

class SequentialImpulsesSolver
{
private:
	void ApplyImpulseAt(ContactConstraint& constraint)
	{
		auto prevAccImpulse = constraint.GetAccumulatedImpulse();

		auto delta = constraint.CalculateImpulse();
		constraint.SetAccumulatedImpulse(MathU::Max(prevAccImpulse + delta, 0.0f));

		auto change = constraint.GetAccumulatedImpulse() - prevAccImpulse;
		constraint.ApplyImpulse(change);
	}

public:
	void Solve(std::vector<ContactConstraint>& contactc)
	{
		for (auto i = 0; i < 10; i++)
		{
			for (auto& c : contactc)
				ApplyImpulseAt(c);
		}
	}
};
