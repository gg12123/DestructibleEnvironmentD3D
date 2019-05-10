#include "pch.h"
#include "Constraints.h"
#include "PhysicsObject.h"

void VelocityAtPointConstraint::RecalculateDenom()
{
	auto& t1 = m_Body1->GetTransform();
	auto& t2 = m_Body2->GetTransform();

	auto s1 = CalculateS(m_Direction, m_Point - t1.GetPosition(), m_Body1->GetInertiaInverseWorld());
	auto s2 = CalculateS(m_Direction, m_Point - t2.GetPosition(), m_Body2->GetInertiaInverseWorld());

	m_Denom = (m_Body1->GetInvMass() + m_Body2->GetInvMass() + s1 + s2);
}

float VelocityAtPointConstraint::CalculateRelativeVelocity() const
{
	auto v1 = m_Body1->WorldVelocityAt(m_Point);
	auto v2 = m_Body2->WorldVelocityAt(m_Point);
	auto vr = v2 - v1;

	return Vector3::Dot(vr, m_Direction);
}

void VelocityAtPointConstraint::ApplyImpulse(float J) const
{
	m_Body1->ApplyImpulse(Impulse(-J * m_Direction, m_Point));
	m_Body2->ApplyImpulse(Impulse(J * m_Direction, m_Point));
}

NormalContactConstraint::NormalContactConstraint(const Shape& s1, const Shape& s2, const Vector3& normal, const Vector3& point, float pen) :
	VelocityAtPointConstraint(*s1.GetOwner().ToPhysicsObject(), *s2.GetOwner().ToPhysicsObject(), normal, point,
	(Beta / PhysicsTime::FixedDeltaTime) * MathU::Max(pen - Slop, 0.0f)),
	m_Penetration(pen)
{
	auto refN = GetBody2().GetTransform().ToWorldPosition(s2.GetCentre()) - GetBody1().GetTransform().ToWorldPosition(s1.GetCentre());
	OrientateDirection(refN);
}

FrictionContactConstraint::FrictionContactConstraint(const Shape& s1, const Shape& s2, const Vector3& manCentre, const Vector3& dir) :
	VelocityAtPointConstraint(*s1.GetOwner().ToPhysicsObject(), *s2.GetOwner().ToPhysicsObject(), dir, manCentre, 0.0f)
{
}

float RotationalJointCostraint::CalculateCurrrentImpulse()
{
	auto deltaV = m_B2->GetAngularVelocity() - m_B1->GetAngularVelocity();
	return -Vector3::Dot(m_V, deltaV) / m_Denom;
}

void RotationalJointCostraint::ApplyImpulse(float imp)
{
	m_B1->ApplyAngularImpulse(-imp * m_V);
	m_B2->ApplyAngularImpulse(imp * m_V);
}

void RotationalJointCostraint::ReCalculateDenom()
{
	m_Denom = Vector3::Dot(m_V, m_B2->GetInertiaInverseLocal() * m_V + m_B1->GetInertiaInverseWorld() * m_V);
}