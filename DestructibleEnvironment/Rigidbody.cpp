#include "pch.h"
#include "Rigidbody.h"
#include "PhysicsTime.h"
#include "MathU.h"
#include "Shape.h"

void Rigidbody::InitMassProperties(const Transform& refTran)
{
	auto mass = 1.0f;
	auto inertia = Matrix3::Indentity();
	auto centreOfMass = Vector3::Zero();

	SetMass(mass);
	SetInertia(inertia);
	CentreAndCache(refTran, centreOfMass);
}

void Rigidbody::UpdateTransform()
{
	auto& t = GetTransform();
	auto& q = t.GetRotation();

	auto pos = t.GetPosition() + m_VelocityWorld * PhysicsTime::FixedDeltaTime;
	auto rot = q + m_AngularVelocityWorld * q * 0.5f * PhysicsTime::FixedDeltaTime;
	
	t.SetPositionAndRotation(pos, rot);
}

void Rigidbody::ApplyImpulse(const Impulse& impulse)
{
	m_VelocityWorld += GetInvMass() * impulse.WorldImpulse;

	auto& r = impulse.WorldImpulsePoint - GetTransform().GetPosition();
	auto& J = impulse.WorldImpulse;

	m_AngularVelocityWorld += (GetInertiaInverseWorld() * Vector3::Cross(r, J));

	static constexpr auto impactNeededForSplit = 1000.0f;

	if (impulse.Impact > impactNeededForSplit)
	{
		m_IsSplit = true;
		m_SplittingImpulse = impulse;
	}
}

void Rigidbody::ApplyExternalForcesAndImpulses()
{
	static constexpr float g = 9.8f;

	m_ExternalForceWorld -= GetMass() * g * Vector3::Up();
	m_ExternalForceWorld -= m_Drag * m_VelocityWorld;

	m_ExternalMomentsWorld -= m_AngularDrag * m_AngularVelocityWorld;

	m_VelocityWorld += (GetInvMass() * m_ExternalForceWorld) * PhysicsTime::FixedDeltaTime;
	m_AngularVelocityWorld += (GetInertiaInverseWorld() * m_ExternalMomentsWorld) * PhysicsTime::FixedDeltaTime;

	for (auto& imp : m_ExternalImpulses)
		ApplyImpulse(imp);

	m_ExternalForceWorld = Vector3::Zero();
	m_ExternalMomentsWorld = Vector3::Zero();
	m_ExternalImpulses.clear();
}

void Rigidbody::UpdatePosition()
{
	UpdateTransform();
	UpdateSubShapesWorldAABBs();
}