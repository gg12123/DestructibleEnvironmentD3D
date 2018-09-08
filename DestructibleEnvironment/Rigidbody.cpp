#include "pch.h"
#include "Rigidbody.h"
#include "PhysicsTime.h"

void Rigidbody::ApplyImpulses(std::vector<SplitInfo>& splits)
{
	Impulse* biggest = nullptr;
	auto biggestImpact = 0.0f;

	for  (auto it = m_Impulses.begin(); it != m_Impulses.end(); it++)
	{
		auto& imp = **it;
		auto impact = imp.Impact;

		ApplyImpulse(imp);

		if (impact > biggestImpact)
		{
			biggest = &imp;
			biggestImpact = impact;
		}
	}

	m_Impulses.clear();

	if (biggest && (biggest->Impact > GetMass()))
	{
		splits.emplace_back(SplitInfo(*this, *biggest));
	}
}

void Rigidbody::UpdateTransform()
{
	auto& t = GetTransform();

	t.SetPosition(t.GetPosition() + m_VelocityWorld * PhysicsTime::FixedDeltaTime + m_ToSeperate);
	m_ToSeperate = Vector3::Zero();

	auto& q = t.GetRotation();
	t.SetRotation(q + q * m_AngularVelocityLocal * (0.5f * PhysicsTime::FixedDeltaTime));
}

void Rigidbody::ApplyImpulse(const Impulse& impulse)
{
	m_VelocityWorld += impulse.WorldImpulse / GetMass();

	auto r = impulse.LocalCollisionPoint;
	auto J = impulse.LocalImpulse;

	m_AngularVelocityLocal += (GetInertiaInverse() * Vector3::Cross(r, J));
}

void Rigidbody::CalculateForces()
{
	static constexpr float g = 9.8f;

	m_AddedForceWorld -= GetMass() * g * Vector3::Up();
	m_AddedForceWorld -= m_Drag * m_VelocityWorld;

	m_AddedMomentsLocal -= m_AngularDrag * m_AngularVelocityLocal;
}

void  Rigidbody::Integrate()
{
	m_VelocityWorld += (m_AddedForceWorld / GetMass()) * PhysicsTime::FixedDeltaTime;
	m_AngularVelocityLocal += (GetInertiaInverse() * m_AddedMomentsLocal) * PhysicsTime::FixedDeltaTime;
}

void Rigidbody::ApplyNormalForces()
{
	CalculateForces();
	Integrate();

	m_AddedForceWorld = Vector3::Zero();
	m_AddedMomentsLocal = Vector3::Zero();
}

void Rigidbody::Update(std::vector<SplitInfo>& splits)
{
	ApplyNormalForces();
	ApplyImpulses(splits);
	UpdateTransform();
}