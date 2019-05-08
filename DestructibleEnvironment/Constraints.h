#pragma once
#include "PhysicsObject.h"
#include "Shape.h"
#include "CollisionData.h"
#include "Face.h"
#include "PhysicsTime.h"

class VelocityAtPointConstraint
{
private:
	static float CalculateS(const Vector3& n, const Vector3& r, const Matrix3 inertiaInverse)
	{
		auto x = inertiaInverse * Vector3::Cross(r, n);
		return Vector3::Dot(n, Vector3::Cross(x, r));
	}

	void RecalculateDenom()
	{
		auto& t1 = m_Body1->GetTransform();
		auto& t2 = m_Body2->GetTransform();

		auto s1 = CalculateS(m_Direction, m_Point - t1.GetPosition(), m_Body1->GetInertiaInverseWorld());
		auto s2 = CalculateS(m_Direction, m_Point - t2.GetPosition(), m_Body2->GetInertiaInverseWorld());

		m_Denom = (m_Body1->GetInvMass() + m_Body2->GetInvMass() + s1 + s2);
	}

public:
	VelocityAtPointConstraint(PhysicsObject& body1, PhysicsObject& body2, const Vector3& direction, const Vector3& point, float velBias)
		: m_Body1(&body1), m_Body2(&body2), m_Direction(direction), m_Point(point), m_AcumulatedImpulse(0.0f), m_VBias(velBias)
	{
		RecalculateDenom();
	}

	VelocityAtPointConstraint(PhysicsObject& body1, PhysicsObject& body2, float velBias)
		: m_Body1(&body1), m_Body2(&body2), m_AcumulatedImpulse(0.0f), m_VBias(velBias)
	{
		RecalculateDenom();
	}

	void ResetPointAndDirection(const Vector3& p, const Vector3& dir)
	{
		m_Point = p;
		m_Direction = dir;
		RecalculateDenom();
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

	void WarmStart()
	{
		ApplyImpulse(m_AcumulatedImpulse);
	}

	void SetVBias(float vBias)
	{
		m_VBias = vBias;
	}

protected:
	void OrientateDirection(const Vector3& refDir)
	{
		m_Direction = m_Direction.InDirectionOf(refDir);
	}

	void ApplyImpulse(float J) const
	{
		m_Body1->ApplyImpulse(Impulse(-J * m_Direction, m_Point));
		m_Body2->ApplyImpulse(Impulse(J * m_Direction, m_Point));
	}

	float CalculateCurrentImpulse() const
	{
		return (-CalculateRelativeVelocity() + m_VBias) / m_Denom;
	}

private:
	PhysicsObject * m_Body1;
	PhysicsObject * m_Body2;
	Vector3 m_Direction;
	Vector3 m_Point;
	float m_Denom;
	float m_AcumulatedImpulse;
	float m_VBias;
};

class NormalContactConstraint : public VelocityAtPointConstraint
{
private:
	static constexpr auto Beta = 0.1f;
	static constexpr auto Slop = 0.01f;

public:
	NormalContactConstraint(const Shape& s1, const Shape& s2, const Vector3& normal, const Vector3& point, float pen) :
		VelocityAtPointConstraint(*s1.GetOwner().ToPhysicsObject(), *s2.GetOwner().ToPhysicsObject(), normal, point,
		(Beta / PhysicsTime::FixedDeltaTime) * MathU::Max(pen - Slop, 0.0f)),
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

	auto GetPenetration() const
	{
		return m_Penetration;
	}

private:
	float m_Penetration;
};

class FrictionContactConstraint : public VelocityAtPointConstraint
{
public:
	FrictionContactConstraint(const Shape& s1, const Shape& s2, const Vector3& manCentre, const Vector3& dir) :
		VelocityAtPointConstraint(*s1.GetOwner().ToPhysicsObject(), *s2.GetOwner().ToPhysicsObject(), dir, manCentre, 0.0f)
	{
	}

	float ApplyNextImpulse(float Jn)
	{
		auto prevAccImpulse = GetAccumulatedImpulse();

		auto delta = CalculateCurrentImpulse();

		static constexpr float mu = 0.5f;
		SetAccumulatedImpulse(MathU::Clamp(prevAccImpulse + delta, -mu * Jn, mu * Jn));

		auto change = GetAccumulatedImpulse() - prevAccImpulse;
		ApplyImpulse(change);

		return change;
	}
};


class JointConstraint : public VelocityAtPointConstraint
{
public:
	JointConstraint(PhysicsObject& b1, PhysicsObject& b2) :
		VelocityAtPointConstraint(b1, b2, 0.0f) // TODO - vel bias is needed for error correction
	{
	}

	float ApplyNextImpulse()
	{
		auto delta = CalculateCurrentImpulse();
		SetAccumulatedImpulse(GetAccumulatedImpulse() + delta);
		ApplyImpulse(delta);
		return delta;
	}
};