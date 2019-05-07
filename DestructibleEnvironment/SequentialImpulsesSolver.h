#pragma once
#include "StringU.h"
#include "Constraints.h"
#include "Islands.h"
#include "ContactManifold.h"
#include "Joint.h"

class SequentialImpulsesSolver
{
private:
	float IterateOnce(SimdStdVector<NormalContactConstraint>& normalConstraints, SimdStdVector<ContactManifold>& manifolds,
		SimdStdVector<Joint>& joints,
		const Island& island) const
	{
		auto impSum = 0.0f;

		for (auto maniIndex : island.GetManifolds())
		{
			auto& m = manifolds[maniIndex];

			auto& range = m.GetNormalConstraintRange();
			auto aveJnAcc = 0.0f;

			for (auto i = range.Start; i < range.End; i++)
			{
				auto& cp = normalConstraints[i];

				impSum += MathU::Abs(cp.ApplyNextImpulse());
				aveJnAcc += cp.GetAccumulatedImpulse();
			}

			aveJnAcc /= static_cast<float>(range.Size());

			impSum += MathU::Abs(m.ApplyFrictionAImpulse(aveJnAcc));
			impSum += MathU::Abs(m.ApplyFrictionBImpulse(aveJnAcc));
		}

		for (auto jointIndex : island.GetJoints())
		{
			auto& joint = joints[jointIndex];
			impSum += joint.ApplyImpulses();
		}

		return impSum;
	}

	void Solve(SimdStdVector<NormalContactConstraint>& normalConstraints, SimdStdVector<ContactManifold>& manifolds,
		SimdStdVector<Joint>& joints, const Island& island) const
	{
		if (island.IsFloatingSingleton() || !island.IsAwake())
			return;

		static constexpr auto maxNumIts = 20;
		static constexpr auto requiredAccuracy = 0.0001f;

		for (auto maniIndex : island.GetManifolds())
			manifolds[maniIndex].WarmStart(normalConstraints);

		for (auto jointIndex : island.GetJoints())
			joints[jointIndex].WarmStart();

		auto impulsesSum = MathU::Infinity;
		auto currIt = 0;

		while (impulsesSum > requiredAccuracy && currIt < maxNumIts)
		{
			impulsesSum = IterateOnce(normalConstraints, manifolds, joints, island);
			currIt++;
		}
	}

public:
	void Solve(SimdStdVector<NormalContactConstraint>& contacts, SimdStdVector<ContactManifold>& manifolds, SimdStdVector<Joint> joints,
		const std::vector<Island*>& islands) const
	{
		for (auto i : islands)
			Solve(contacts, manifolds, joints, *i);
	}
};
