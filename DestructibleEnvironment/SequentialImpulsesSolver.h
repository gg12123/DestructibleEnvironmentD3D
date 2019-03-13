#pragma once
#include "ContactPoints.h"
#include "CollisionResponder.h"

class SequentialImpulsesSolver
{
public:
	void Solve(std::vector<NormalContactConstraint>& contacts, std::vector<ContactManifold>& manifolds)
	{
		for (auto i = 0; i < 10; i++)
		{
			for (auto& c : manifolds)
				c.ApplyImpulses(contacts);
		}
	}
};
