#pragma once
#include "PhysicsObject.h"
#include "Shape.h"
#include "CollisionData.h"
#include "Face.h"
#include "PhysicsTime.h"
#include "Constraints.h"
#include "Debug.h"
#include "ContactContexts.h"

class ContactManifold
{
public:
	ContactManifold(const Shape& s1, const Shape& s2,
		const StdVectorRange& normalConstraintRange,
		const Vector3& dirA, const Vector3& dirB, const Vector3& centre, const Vector3& frictionWarmStart) :
		m_NormalConstraintRange(normalConstraintRange),
		m_FrictionA(s1, s2, centre, dirA), m_FrictionB(s1, s2, centre, dirB),
		m_Shape1(&s1), m_Shape2(&s2)
	{
		m_FrictionA.WarmStart(Vector3::Dot(frictionWarmStart, m_FrictionA.GetDirection()));
		m_FrictionB.WarmStart(Vector3::Dot(frictionWarmStart, m_FrictionB.GetDirection()));
	}

	const auto& GetNormalConstraintRange() const
	{
		return m_NormalConstraintRange;
	}

	const auto& GetShape1() const
	{
		return *m_Shape1;
	}

	const auto& GetShape2() const
	{
		return *m_Shape2;
	}

	auto GetAccumulatedFrictionImpulse() const
	{
		return m_FrictionA.GetAccumulatedImpulse() * m_FrictionA.GetDirection() +
			m_FrictionB.GetAccumulatedImpulse() * m_FrictionB.GetDirection();
	}

	auto ApplyFrictionAImpulse(float aveJnAcc)
	{
		return m_FrictionA.ApplyNextImpulse(aveJnAcc);
	}

	auto ApplyFrictionBImpulse(float aveJnAcc)
	{
		return m_FrictionB.ApplyNextImpulse(aveJnAcc);
	}

private:
	StdVectorRange m_NormalConstraintRange;
	int m_End;
	FrictionContactConstraint m_FrictionA;
	FrictionContactConstraint m_FrictionB;
	const Shape* m_Shape1;
	const Shape* m_Shape2;
};

class ManifoldInitializer
{
private:
	struct StoredAccImpulse
	{
		float Impulse;
		Vector3 ContactPoint;

		StoredAccImpulse(const Vector3& p, float impulse)
		{
			ContactPoint = p;
			Impulse = impulse;
		}
	};

	struct PerManifoldHistoryData
	{
		Vector3 FrictionAccImpulse;
		float Penetration;
		Vector3 ContactNormal;

		PerManifoldHistoryData(const Vector3& frictionAccImpulse, float pen, const Vector3& n)
		{
			FrictionAccImpulse = frictionAccImpulse;
			Penetration = pen;
			ContactNormal = n;
		}
	};

	ContactManifold SetUpManifold(const Shape& shape1, const Shape& shape2, SimdStdVector<NormalContactConstraint>& constraints, const Vector3& warmStartFriction) const
	{
		auto centre = Vector3::Zero();
		auto end = static_cast<int>(constraints.size());

		for (int i = m_StartOfCurrManifold; i < end; i++)
		{
			centre += constraints[i].GetPoint();
		}

		centre /= static_cast<float>(end - m_StartOfCurrManifold);

		auto n = constraints[m_StartOfCurrManifold].GetDirection();

		auto vr = shape1.GetOwner().ToPhysicsObject()->WorldVelocityAt(centre) -
			shape2.GetOwner().ToPhysicsObject()->WorldVelocityAt(centre);

		auto vrProj = Vector3::ProjectOnPlane(n, vr);

		auto dirA = vrProj.MagnitudeSqr() > 0.001f ? vrProj.Normalized() : Vector3::OrthogonalDirection(n);
		auto dirB = Vector3::Cross(n, dirA);

		auto ncRange = StdVectorRange(m_StartOfCurrManifold, end);

		return ContactManifold(shape1, shape2, ncRange, dirA, dirB, centre, warmStartFriction);
	}

	float FindWarmStartAccImpulse(const ContactContext& context, const Vector3& cp, const Vector3& n) const
	{
		auto& range = context.ContactPointsRange;
		auto imp = 0.0f;
		auto minDist = MathU::Infinity;

		static constexpr auto tol = 0.01f;

		for (auto i = range.Start; i < range.End; i++)
		{
			auto dist = Vector3::ProjectOnPlane(n, cp - m_NormalAccImpulses[i].ContactPoint).MagnitudeSqr();
			if (dist < minDist && dist < tol)
			{
				imp = m_NormalAccImpulses[i].Impulse;
				minDist = dist;
			}
		}
		return imp;
	}

public:
	ManifoldInitializer()
	{
	}

	void InitManifoldUsingPrevContactPoints(const Shape& shape1, const Shape& shape2, const ContactContext& context)
	{
		auto& prevManifold = m_PerManifoldHistory[context.ManifoldHistoryIndex];

		auto& n = prevManifold.ContactNormal;
		auto& pen = prevManifold.Penetration;

		auto& range = context.ContactPointsRange;
		for (auto i = range.Start; i < range.End; i++)
		{
			auto& storedImpulse = m_NormalAccImpulses[i];

			auto nc = NormalContactConstraint(shape1, shape2, n, storedImpulse.ContactPoint, pen);
			nc.WarmStart(storedImpulse.Impulse);

			m_NormalContactContraints.emplace_back(nc);
		}

		auto& frictionWarmStart = m_PerManifoldHistory[context.ManifoldHistoryIndex].FrictionAccImpulse;

		m_Manifolds.emplace_back(SetUpManifold(shape1, shape2, m_NormalContactContraints, frictionWarmStart));
	}

	void InitManifold(const Shape& shape1, const Shape& shape2,
		const SimdStdVector<Vector3>& contactPoints,
		const ContactPlane& contactPlane,
		const ContactContext& context)
	{
		static constexpr auto doWarmStartNormal = true;
		static constexpr auto doWarmStartFriction = true;

		if (contactPoints.size() == 0u)
		{
			Debug::Log(std::string("ERROR - init manifold called with zero contact points."));
			return;
		}

		m_StartOfCurrManifold = m_NormalContactContraints.size();

		auto n = contactPlane.GetNormal();
		auto pen = contactPlane.GetPeneration();

		if (doWarmStartNormal && context.InContactOnPrevTick())
		{
			// Warm start
			for (auto& p : contactPoints)
			{
				auto nc = NormalContactConstraint(shape1, shape2, n, p, pen);
				nc.WarmStart(FindWarmStartAccImpulse(context, p, n));

				m_NormalContactContraints.emplace_back(nc);
			}
		}
		else
		{
			// These shapes were not in contact on prev tick so cannot warm start
			for (auto& p : contactPoints)
			{
				m_NormalContactContraints.emplace_back(NormalContactConstraint(shape1, shape2, n, p, pen));
			}
		}

		auto frictionWarmStart = doWarmStartFriction && context.InContactOnPrevTick() ?
			m_PerManifoldHistory[context.ManifoldHistoryIndex].FrictionAccImpulse :
			Vector3::Zero();

		m_Manifolds.emplace_back(SetUpManifold(shape1, shape2, m_NormalContactContraints, frictionWarmStart));
	}

	void StoreAccumulatedImpulsesForNextTick(ContactContexts& contexts)
	{
		m_PerManifoldHistory.clear();
		m_NormalAccImpulses.clear();

		// Could optimise this with a memcpy or something. This is definitly not bottleneck
		// though so probably doesnt matter.
		for (auto& nc : m_NormalContactContraints)
			m_NormalAccImpulses.emplace_back(StoredAccImpulse(nc.GetPoint(), nc.GetAccumulatedImpulse()));

		for (auto& m : m_Manifolds)
		{
			auto& context = contexts.GetContext(m.GetShape1(), m.GetShape2());
			context.ContactPointsRange = m.GetNormalConstraintRange();
			context.ManifoldHistoryIndex = m_PerManifoldHistory.size();

			auto& n = m_NormalContactContraints[context.ContactPointsRange.Start];

			m_PerManifoldHistory.emplace_back(PerManifoldHistoryData(m.GetAccumulatedFrictionImpulse(),
				n.GetPenetration(), n.GetDirection()));
		}

		m_Manifolds.clear();
		m_NormalContactContraints.clear();
	}

	auto& GetManifolds()
	{
		return m_Manifolds;
	}

	auto& GetNormalContactConstraints()
	{
		return m_NormalContactContraints;
	}

private:
	int m_StartOfCurrManifold;

	SimdStdVector<NormalContactConstraint> m_NormalContactContraints;
	SimdStdVector<ContactManifold> m_Manifolds;

	// Per contact point history - keyed by context.ContactPointsRange
	SimdStdVector<StoredAccImpulse> m_NormalAccImpulses;

	// Per manifold history - keyed by context.ManifoldHistoryIndex
	SimdStdVector<PerManifoldHistoryData> m_PerManifoldHistory;
};
