#pragma once
#include "PhysicsObject.h"
#include "Shape.h"
#include "CollisionData.h"
#include "CollisionResponder.h"
#include "Face.h"
#include "PhysicsTime.h"
#include "Constraints.h"
#include "Debug.h"
#include "DynamicTriangleArray.h"

struct StdVectorRange
{
	int Start;
	int End;

	int Size() const
	{
		return End - Start;
	}

	StdVectorRange(int s, int e)
	{
		Start = s;
		End = e;
	}

	StdVectorRange() : Start(0), End(0)
	{
	}
};

class ContactManifold
{
public:
	ContactManifold(const Shape& s1, const Shape& s2,
		const StdVectorRange& normalConstraintRange,
		const Vector3& dirA, const Vector3& dirB, const Vector3& centre) :
		m_NormalConstraintRange(normalConstraintRange),
		m_FrictionA(s1, s2, centre, dirA), m_FrictionB(s1, s2, centre, dirB),
		m_Shape1(&s1), m_Shape2(&s2)
	{
	}

	float ApplyImpulses(std::vector<NormalContactConstraint>& contactPoints)
	{
		auto aveJnAcc = 0.0f;
		auto maxImp = 0.0f;

		for (auto i = m_NormalConstraintRange.Start; i < m_NormalConstraintRange.End; i++)
		{
			auto& cp = contactPoints[i];
			maxImp = MathU::Max(MathU::Abs(cp.ApplyNextImpulse()), maxImp);

			aveJnAcc += cp.GetAccumulatedImpulse();
		}

		aveJnAcc /= static_cast<float>(m_NormalConstraintRange.Size());

		maxImp = MathU::Max(MathU::Abs(m_FrictionA.ApplyNextImpulse(aveJnAcc)), maxImp);
		maxImp = MathU::Max(MathU::Abs(m_FrictionB.ApplyNextImpulse(aveJnAcc)), maxImp);

		return maxImp;
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
	struct ManifoldContext
	{
		uint64 TimeStamp = 0;
		StdVectorRange ContactPointsRange;
	};

	struct StoredAccImpulse
	{
		float Impulse;
		Vector3 ContactPoint;

		StoredAccImpulse(const Vector3& p)
		{
			ContactPoint = p;
		}
	};

	ContactManifold SetUpManifold(const Shape& shape1, const Shape& shape2, std::vector<NormalContactConstraint>& constraints) const
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

		return ContactManifold(shape1, shape2, ncRange, dirA, dirB, centre);
	}

	ManifoldContext* GetContext(const Shape& shape1, const Shape& shape2)
	{
		auto& c = ForceGetContext(shape1, shape2);

		if (c.TimeStamp == m_CurrTimeStamp)
			return &c;

		return nullptr;
	}

	ManifoldContext& ForceGetContext(const Shape& shape1, const Shape& shape2)
	{
		return m_Contexts.Get(shape1.GetShapeId(), shape2.GetShapeId());
	}

	float FindWarmStartAccImpulse(const ManifoldContext& context, const Vector3& cp, const Vector3& n) const
	{
		auto& range = context.ContactPointsRange;
		auto imp = 0.0f;
		auto minDist = MathU::Infinity;

		static constexpr auto tol = 0.01f;

		for (auto i = range.Start; i < range.End; i++)
		{
			auto dist = Vector3::ProjectOnPlane(n, cp - m_AccImpulses[i].ContactPoint).MagnitudeSqr();
			if (dist < minDist && dist < tol)
			{
				imp = m_AccImpulses[i].Impulse;
				minDist = dist;
			}
		}

		return imp;
	}

public:
	ManifoldInitializer() : m_CurrTimeStamp(1)
	{
	}

	void InitManifold(const Shape& shape1, const Shape& shape2,
		const std::vector<Vector3>& contactPoints,
		const ContactPlane& contactPlane)
	{
		if (contactPoints.size() == 0u)
		{
			Debug::Log(std::string("ERROR - init manifold called with zero contact points."));
			return;
		}

		m_StartOfCurrManifold = m_NormalContactContraints.size();

		auto n = contactPlane.GetNormal();
		auto pen = contactPlane.GetPeneration();

		static constexpr auto doWarmStart = true;

		auto context = GetContext(shape1, shape2);
		if (doWarmStart && context)
		{
			// Warm start
			for (auto& p : contactPoints)
			{
				auto nc = NormalContactConstraint(shape1, shape2, n, p, pen);
				nc.WarmStart(FindWarmStartAccImpulse(*context, p, n));

				m_NormalContactContraints.emplace_back(nc);
				m_NextAccImpulses.emplace_back(StoredAccImpulse(p));
			}
		}
		else
		{
			// These shapes were not in contact on prev tick so cannot warm start
			for (auto& p : contactPoints)
			{
				m_NormalContactContraints.emplace_back(NormalContactConstraint(shape1, shape2, n, p, pen));
				m_NextAccImpulses.emplace_back(StoredAccImpulse(p));
			}
		}

		m_Manifolds.emplace_back(SetUpManifold(shape1, shape2, m_NormalContactContraints));
	}

	void StoreAccumulatedImpulsesForNextTick()
	{
		m_CurrTimeStamp++;

		// TODO - As uint64 is so big I dont think this will ever throw.
		// Must check though.
		assert(m_CurrTimeStamp < (std::numeric_limits<uint64>::max)());

		for (auto& m : m_Manifolds)
		{
			auto& ncRange = m.GetNormalConstraintRange();

			for (auto i = ncRange.Start; i < ncRange.End; i++)
			{
				m_NextAccImpulses[i].Impulse = m_NormalContactContraints[i].GetAccumulatedImpulse();
			}

			auto& context = ForceGetContext(m.GetShape1(), m.GetShape2());
			context.ContactPointsRange = ncRange;
			context.TimeStamp = m_CurrTimeStamp;
		}

		m_AccImpulses.swap(m_NextAccImpulses);

		m_NextAccImpulses.clear();
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

	std::vector<NormalContactConstraint> m_NormalContactContraints;
	std::vector<ContactManifold> m_Manifolds;

	std::vector<StoredAccImpulse> m_AccImpulses;
	std::vector<StoredAccImpulse> m_NextAccImpulses;

	uint64 m_CurrTimeStamp;
	DynamicTriangleArray<ManifoldContext> m_Contexts;
};
