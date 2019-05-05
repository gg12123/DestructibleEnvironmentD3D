#pragma once
#include "StringU.h"
#include "Constraints.h"
#include "Islands.h"

class SequentialImpulsesSolver
{
private:
	float IterateOnce(SimdStdVector<NormalContactConstraint>& contacts, SimdStdVector<ContactManifold>& manifolds, const Island& island) const
	{
		auto impSum = 0.0f;

		for (auto maniIndex : island.GetManifolds())
		{
			auto& m = manifolds[maniIndex];

			auto& range = m.GetNormalConstraintRange();
			auto aveJnAcc = 0.0f;

			for (auto i = range.Start; i < range.End; i++)
			{
				auto& cp = contacts[i];

				impSum += MathU::Abs(cp.ApplyNextImpulse());
				aveJnAcc += cp.GetAccumulatedImpulse();
			}

			aveJnAcc /= static_cast<float>(range.Size());

			impSum += MathU::Abs(m.ApplyFrictionAImpulse(aveJnAcc));
			impSum += MathU::Abs(m.ApplyFrictionBImpulse(aveJnAcc));
		}

		return impSum;
	}

	void Solve(SimdStdVector<NormalContactConstraint>& contacts, SimdStdVector<ContactManifold>& manifolds, const Island& island) const
	{
		if (island.IsSingleton() || !island.IsAwake())
			return;

		static constexpr auto maxNumIts = 20;
		static constexpr auto requiredAccuracy = 0.0001f;

		auto impulsesSum = MathU::Infinity;
		auto currIt = 0;

		while (impulsesSum > requiredAccuracy && currIt < maxNumIts)
		{
			impulsesSum = IterateOnce(contacts, manifolds, island);
			currIt++;
		}

		//Debug::Log("Num its required: " + StringU::ToString(currIt));
		//Debug::Log("Convergence: " + StringU::ToString(impulsesSum));
	}

public:
	void Solve(SimdStdVector<NormalContactConstraint>& contacts, SimdStdVector<ContactManifold>& manifolds, const std::vector<Island*>& islands) const
	{
		for (auto i : islands)
			Solve(contacts, manifolds, *i);
	}
};
