#pragma once
#include "CollisionResponder.h"
#include "StringU.h"

class SequentialImpulsesSolver
{
public:
	void Solve(std::vector<NormalContactConstraint>& contacts, std::vector<ContactManifold>& manifolds)
	{
		if (manifolds.size() == 0u)
			return;

		for (auto i = 0; i < 10; i++)
		{
			auto impulsesSum = 0.0f;
			for (auto& c : manifolds)
			{
				impulsesSum += c.ApplyImpulses(contacts);
			}

			Debug::Log("Iteration " + StringU::ToString(i) + ": " + StringU::ToString(impulsesSum));
		}
	}
};
