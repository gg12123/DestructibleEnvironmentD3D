#pragma once
#include "PhysicsObject.h"
#include "Shape.h"
#include "CollisionData.h"
#include "CollisionResponder.h"
#include "Face.h"
#include "PhysicsTime.h"
#include "Constraints.h"
#include "ContactPoints.h"
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
};

class ContactManifold
{
public:
	ContactManifold(const Shape& s1, const Shape& s2,
		const StdVectorRange& normalConstraintRange, const StdVectorRange& contactPointEcRange, const StdVectorRange& contactPointPipRange,
		const Vector3& dirA, const Vector3& dirB, const Vector3& centre) :
		m_NormalConstraintRange(normalConstraintRange), m_ContactPointEcRange(contactPointEcRange),
		m_ContactPointPipRange(contactPointPipRange),
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

	const auto& GetContactPointEcRange() const
	{
		return m_ContactPointEcRange;
	}

	const auto& GetContactPointPipRange() const
	{
		return m_ContactPointPipRange;
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
	StdVectorRange m_ContactPointEcRange;
	StdVectorRange m_ContactPointPipRange;
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
		StdVectorRange ContactPointsEcRange;
		StdVectorRange ContactPointsPipRange;
	};

	struct StoredAccImpulseEC
	{
		float Impulse;
		int EdgeIndexShape1;
		int EdgeIndexShape2;

		StoredAccImpulseEC(int i1, int i2)
		{
			EdgeIndexShape1 = i1;
			EdgeIndexShape2 = i2;
		}
	};

	struct StoredAccImpulsePIP
	{
		float Impulse;
		int PointIndex;
		int ShapeId;

		StoredAccImpulsePIP(int pi, int sid)
		{
			PointIndex = pi;
			ShapeId = sid;
		}
	};

	ContactManifold SetUpManifold(const Shape& shape1, const Shape& shape2, std::vector<NormalContactConstraint>& constraints) const
	{
		auto centre = Vector3::Zero();
		auto end = static_cast<int>(constraints.size());

		for (int i = m_StartOfCurrManifoldNC; i < end; i++)
		{
			centre += constraints[i].GetPoint();
		}

		centre /= static_cast<float>(end - m_StartOfCurrManifoldNC);

		auto n = constraints[m_StartOfCurrManifoldNC].GetDirection();

		auto vr = shape1.GetOwner().ToPhysicsObject()->WorldVelocityAt(centre) -
			shape2.GetOwner().ToPhysicsObject()->WorldVelocityAt(centre);

		auto vrProj = Vector3::ProjectOnPlane(n, vr);

		auto dirA = vrProj.MagnitudeSqr() > 0.001f ? vrProj.Normalized() : Vector3::OrthogonalDirection(n);
		auto dirB = Vector3::Cross(n, dirA);

		auto ncRange = StdVectorRange(m_StartOfCurrManifoldNC, end);
		auto ecRange = StdVectorRange(m_StartOfCurrManifoldNextEC, m_NextAccImpulsesEC.size());
		auto pipRange = StdVectorRange(m_StartOfCurrManifoldNextPIP, m_NextAccImpulsesPIP.size());

		return ContactManifold(shape1, shape2, ncRange, ecRange, pipRange, dirA, dirB, centre);
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

	float FindWarmStartAccImpulse(const ManifoldContext& context, const ContactPointEC& cp) const
	{
		auto& range = context.ContactPointsEcRange;
		for (auto i = range.Start; i < range.End; i++)
		{
			auto& prevCp = m_AccImpulsesEC[i];
			if (prevCp.EdgeIndexShape1 == cp.EdgeIndexShape1 && prevCp.EdgeIndexShape2 == cp.EdgeIndexShape2)
				return prevCp.Impulse;
		}
		return 0.0f;
	}

	float FindWarmStartAccImpulse(const ManifoldContext& context, const ContactPointPIP& cp) const
	{
		auto& range = context.ContactPointsEcRange;
		for (auto i = range.Start; i < range.End; i++)
		{
			auto& prevCp = m_AccImpulsesPIP[i];
			if (prevCp.PointIndex == cp.PointIndex && prevCp.ShapeId == cp.ShapeId)
				return prevCp.Impulse;
		}
		return 0.0f;
	}

public:
	ManifoldInitializer() : m_CurrTimeStamp(1)
	{
	}

	void InitManifold(const Shape& shape1, const Shape& shape2,
		const std::vector<ContactPointEC>& contactPointsEC, const std::vector<ContactPointPIP>& contactPointsPIP,
		const ContactPlane& contactPlane)
	{
		if (contactPointsEC.size() + contactPointsPIP.size() == 0u)
		{
			Debug::Log(std::string("ERROR - init manifold called with zero contact points."));
			return;
		}

		m_StartOfCurrManifoldNC = m_NormalContactContraints.size();
		m_StartOfCurrManifoldNextEC = m_NextAccImpulsesEC.size();
		m_StartOfCurrManifoldNextPIP = m_NextAccImpulsesPIP.size();

		auto n = contactPlane.GetNormal();
		auto pen = contactPlane.GetPeneration();

		auto context = GetContext(shape1, shape2);
		if (context)
		{
			// Warm start
			for (auto& p : contactPointsEC)
			{
				auto nc = NormalContactConstraint(shape1, shape2, n, p.Point, pen);
				nc.SetAccumulatedImpulse(FindWarmStartAccImpulse(*context, p));

				m_NormalContactContraints.emplace_back(nc);
				m_NextAccImpulsesEC.emplace_back(StoredAccImpulseEC(p.EdgeIndexShape1, p.EdgeIndexShape2));
			}

			for (auto& p : contactPointsPIP)
			{
				auto nc = NormalContactConstraint(shape1, shape2, n, p.Point, pen);
				nc.SetAccumulatedImpulse(FindWarmStartAccImpulse(*context, p));

				m_NormalContactContraints.emplace_back(nc);
				m_NextAccImpulsesPIP.emplace_back(StoredAccImpulsePIP(p.PointIndex, p.ShapeId));
			}
		}
		else
		{
			// These shapes were not in contact on prev tick so cannot warm start

			for (auto& p : contactPointsEC)
			{
				m_NormalContactContraints.emplace_back(NormalContactConstraint(shape1, shape2, n, p.Point, pen));
				m_NextAccImpulsesEC.emplace_back(StoredAccImpulseEC(p.EdgeIndexShape1, p.EdgeIndexShape2));
			}

			for (auto& p : contactPointsPIP)
			{
				m_NormalContactContraints.emplace_back(NormalContactConstraint(shape1, shape2, n, p.Point, pen));
				m_NextAccImpulsesPIP.emplace_back(StoredAccImpulsePIP(p.PointIndex, p.ShapeId));
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
			auto& ecRange = m.GetContactPointEcRange();
			auto& pipRange = m.GetContactPointPipRange();

			auto ncRangeStart = ncRange.Start;

			for (auto i = 0; i < ecRange.Size(); i++)
			{
				m_NextAccImpulsesEC[ecRange.Start + i].Impulse =
					m_NormalContactContraints[ncRangeStart + i].GetAccumulatedImpulse();
			}

			ncRangeStart += ecRange.Size;

			for (auto i = 0; i < pipRange.Size(); i++)
			{
				m_NextAccImpulsesPIP[pipRange.Start + i].Impulse =
					m_NormalContactContraints[ncRangeStart + i].GetAccumulatedImpulse();
			}

			auto& context = ForceGetContext(m.GetShape1(), m.GetShape2());
			context.ContactPointsEcRange = ecRange;
			context.ContactPointsPipRange = pipRange;
			context.TimeStamp = m_CurrTimeStamp;
		}

		m_AccImpulsesEC.swap(m_NextAccImpulsesEC);
		m_AccImpulsesPIP.swap(m_NextAccImpulsesPIP);

		m_NextAccImpulsesEC.clear();
		m_NextAccImpulsesPIP.clear();
	}

private:
	int m_StartOfCurrManifoldNC;
	int m_StartOfCurrManifoldNextEC;
	int m_StartOfCurrManifoldNextPIP;

	std::vector<NormalContactConstraint> m_NormalContactContraints;
	std::vector<ContactManifold> m_Manifolds;

	std::vector<StoredAccImpulseEC> m_AccImpulsesEC;
	std::vector<StoredAccImpulsePIP> m_AccImpulsesPIP;

	std::vector<StoredAccImpulseEC> m_NextAccImpulsesEC;
	std::vector<StoredAccImpulsePIP> m_NextAccImpulsesPIP;

	uint64 m_CurrTimeStamp;
	DynamicTriangleArray<ManifoldContext> m_Contexts;
};
