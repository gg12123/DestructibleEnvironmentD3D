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

	void WarmStart(float accImpulse)
	{
		SetAccumulatedImpulse(accImpulse);
		ApplyImpulse(accImpulse);
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

		static constexpr float mu = 0.5f;
		SetAccumulatedImpulse(MathU::Clamp(prevAccImpulse + delta, -mu * Jn, mu * Jn));

		auto change = GetAccumulatedImpulse() - prevAccImpulse;
		ApplyImpulse(change);

		return change;
	}
};
