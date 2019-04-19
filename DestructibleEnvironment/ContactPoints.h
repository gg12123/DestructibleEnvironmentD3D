#pragma once
#include "PhysicsObject.h"
#include "Shape.h"
#include "CollisionData.h"
#include "CollisionResponder.h"
#include "Face.h"
#include "PhysicsTime.h"

class ContactConstraint
{
private:
	static float CalculateS(const Vector3& n, const Vector3& r, const Matrix3 inertiaInverse)
	{
		auto x = inertiaInverse * Vector3::Cross(r, n);
		return Vector3::Dot(n, Vector3::Cross(x, r));
	}

public:
	ContactConstraint(PhysicsObject& body1, PhysicsObject& body2, const Vector3& direction, const Vector3& point)
		: m_Body1(&body1), m_Body2(&body2), m_Direction(direction), m_Point(point), m_AcumulatedImpulse(0.0f)
	{
		auto& t1 = body1.GetTransform();
		auto& t2 = body2.GetTransform();

		auto s1 = CalculateS(m_Direction, m_Point - t1.GetPosition(), body1.GetInertiaInverseWorld());
		auto s2 = CalculateS(m_Direction, m_Point - t2.GetPosition(), body2.GetInertiaInverseWorld());

		m_Denom = (body1.GetInvMass() + body2.GetInvMass() + s1 + s2);
	}

	auto& GetBody1() const
	{
		return *m_Body1;
	}

	auto& GetBody2() const
	{
		return *m_Body2;
	}

	const auto& GetPoint() const
	{
		return m_Point;
	}

	const auto& GetDirection() const
	{
		return m_Direction;
	}

	auto GetDenom() const
	{
		return m_Denom;
	}

	float GetAccumulatedImpulse() const
	{
		return m_AcumulatedImpulse;
	}

	void SetAccumulatedImpulse(float val)
	{
		m_AcumulatedImpulse = val;
	}

	float CalculateRelativeVelocity() const
	{
		auto v1 = m_Body1->WorldVelocityAt(m_Point);
		auto v2 = m_Body2->WorldVelocityAt(m_Point);
		auto vr = v2 - v1;

		return Vector3::Dot(vr, m_Direction);
	}

protected:
	void OrientateDirection(const Vector3& refDir)
	{
		m_Direction = m_Direction.InDirectionOf(refDir);
	}

	void ApplyImpulse(float J) const
	{
		m_Body1->ApplyImpulse(Impulse(-J * m_Direction, m_Point, J));
		m_Body2->ApplyImpulse(Impulse(J * m_Direction, m_Point, J));
	}

private:
	PhysicsObject * m_Body1;
	PhysicsObject * m_Body2;
	Vector3 m_Direction;
	Vector3 m_Point;
	float m_Denom;
	float m_AcumulatedImpulse;
};

class NormalContactConstraint : public ContactConstraint
{
private:
	float CalculateCurrentImpulse() const
	{
		static constexpr auto beta = 0.1f;
		static constexpr auto slop = 0.01f;
		auto vBias = (beta / PhysicsTime::FixedDeltaTime) * MathU::Max(m_Penetration - slop, 0.0f);

		return (-CalculateRelativeVelocity() + vBias) / GetDenom();
	}

public:
	NormalContactConstraint(const Shape& s1, const Shape& s2, const Vector3& normal, const Vector3& point, float pen) :
		ContactConstraint(*s1.GetOwner().ToPhysicsObject(), *s2.GetOwner().ToPhysicsObject(), normal, point),
		m_Penetration(pen)
	{
		auto refN = GetBody2().GetTransform().ToWorldPosition(s2.GetCentre()) - GetBody1().GetTransform().ToWorldPosition(s1.GetCentre());
		OrientateDirection(refN);
	}

	float ApplyNextImpulse()
	{
		auto prevAccImpulse = GetAccumulatedImpulse();

		auto delta = CalculateCurrentImpulse();
		SetAccumulatedImpulse(MathU::Max(prevAccImpulse + delta, 0.0f));

		auto change = GetAccumulatedImpulse() - prevAccImpulse;
		ApplyImpulse(change);

		return change;
	}

private:
	float m_Penetration;
};

class FrictionContactConstraint : ContactConstraint
{
private:
	float CalculateCurrentImpulse() const
	{
		return (-CalculateRelativeVelocity()) / GetDenom();
	}

public:
	FrictionContactConstraint(const Shape& s1, const Shape& s2, const Vector3& manCentre, const Vector3& dir) :
		ContactConstraint(*s1.GetOwner().ToPhysicsObject(), *s2.GetOwner().ToPhysicsObject(), dir, manCentre)
	{
	}

	float ApplyNextImpulse(float Jn)
	{
		auto prevAccImpulse = GetAccumulatedImpulse();

		auto delta = CalculateCurrentImpulse();
		
		static constexpr float mu = 1.0f;
		SetAccumulatedImpulse(MathU::Clamp(prevAccImpulse + delta, -mu * Jn, mu * Jn));

		auto change = GetAccumulatedImpulse() - prevAccImpulse;
		ApplyImpulse(change);

		return change;
	}
};

class ContactManifold
{
public:
	ContactManifold(const Shape& s1, const Shape& s2, int start, int end, const Vector3& dirA, const Vector3& dirB, const Vector3& centre) :
		m_Start(start), m_End(end), m_FrictionA(s1, s2, centre, dirA), m_FrictionB(s1, s2, centre, dirB)
	{
	}

	float ApplyImpulses(std::vector<NormalContactConstraint>& contactPoints)
	{
		auto aveJnAcc = 0.0f;
		auto maxImp = 0.0f;

		for (auto i = m_Start; i < m_End; i++)
		{
			auto& cp = contactPoints[i];
			maxImp = MathU::Max(MathU::Abs(cp.ApplyNextImpulse()), maxImp);

			aveJnAcc += cp.GetAccumulatedImpulse();
		}

		aveJnAcc /= static_cast<float>(m_End - m_Start);

		maxImp = MathU::Max(MathU::Abs(m_FrictionA.ApplyNextImpulse(aveJnAcc)), maxImp);
		maxImp = MathU::Max(MathU::Abs(m_FrictionB.ApplyNextImpulse(aveJnAcc)), maxImp);

		return maxImp;
	}

private:
	int m_Start;
	int m_End;
	FrictionContactConstraint m_FrictionA;
	FrictionContactConstraint m_FrictionB;
};

class ManifoldInitializer
{
private:
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

		return ContactManifold(shape1, shape2, m_StartOfCurrManifold, end, dirA, dirB, centre);
	}

public:
	void InitManifold(std::vector<NormalContactConstraint>& normalConstraints, std::vector<ContactManifold>& manifolds,
		const Shape& shape1, const Shape& shape2, const std::vector<Vector3>& contactPoints, const ContactPlane& contactPlane)
	{
		if (contactPoints.size() == 0u)
		{
			Debug::Log(std::string("ERROR - init manifold called with zero contact points."));
			return;
		}

		m_StartOfCurrManifold = normalConstraints.size();

		auto n = contactPlane.GetNormal();
		auto pen = contactPlane.GetPeneration();

		for (auto& p : contactPoints)
			normalConstraints.emplace_back(NormalContactConstraint(shape1, shape2, n, p, pen));

		manifolds.emplace_back(SetUpManifold(shape1, shape2, normalConstraints));
	}

private:
	int m_StartOfCurrManifold;
};