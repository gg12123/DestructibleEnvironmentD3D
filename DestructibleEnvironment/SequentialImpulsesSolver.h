#pragma once
#include "StringU.h"
#include "Constraints.h"

class SequentialImpulsesSolver
{
private:
	float Iterate(SimdStdVector<NormalContactConstraint>& contacts, SimdStdVector<ContactManifold>& manifolds) const
	{
		auto impSum = 0.0f;

		for (auto& m : manifolds)
		{
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

public:
	void Solve(SimdStdVector<NormalContactConstraint>& contacts, SimdStdVector<ContactManifold>& manifolds) const
	{
		if (manifolds.size() == 0u)
			return;

		static constexpr auto maxNumIts = 20;
		static constexpr auto requiredAccuracy = 0.0001f;

		auto impulsesSum = MathU::Infinity;
		auto currIt = 0;

		while (impulsesSum > requiredAccuracy && currIt < maxNumIts)
		{
			impulsesSum = Iterate(contacts, manifolds);
			currIt++;
		}

		//Debug::Log("Num its required: " + StringU::ToString(currIt));
		//Debug::Log("Convergence: " + StringU::ToString(impulsesSum));
	}
};
