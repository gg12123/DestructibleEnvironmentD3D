#include "pch.h"
#include "Rigidbody.h"
#include "PhysicsTime.h"

void Rigidbody::CalculateInertia()
{
	auto& points = GetCachedPoints();

	auto m = GetMass();

	auto Ixx = 0.0f;
	auto Iyy = 0.0f;
	auto Izz = 0.0f;

	auto Ixy = 0.0f;
	auto Ixz = 0.0f;
	auto Iyz = 0.0f;

	for (auto it = points.begin(); it != points.end(); it++)
	{
		auto P = (*it);

		Ixx += (P.y * P.y + P.z * P.z) * m;
		Iyy += (P.x * P.x + P.z * P.z) * m;
		Izz += (P.x * P.x + P.y * P.y) * m;

		Ixy += (P.x * P.y) * m;
		Ixz += (P.x * P.z) * m;
		Iyz += (P.y * P.z) * m;
	}

	Matrix3 inertia;

	auto col0 = inertia.M[0];
	col0[0] = Ixx;
	col0[1] = -Ixy;
	col0[2] = -Ixz;

	auto col1 = inertia.M[1];
	col1[0] = -Ixy;
	col1[1] = Iyy;
	col1[2] = -Iyz;

	auto col2 = inertia.M[2];
	col2[0] = -Ixz;
	col2[1] = -Iyz;
	col2[2] = Izz;

	SetInertia(inertia);
}

void Rigidbody::CalculateMotionProperties()
{
	SetMass(GetLocalBounds().GetVolume());
	CalculateInertia();

	// TODO - not sure how to calculate these...
	m_Drag = 1.5f;
	m_AngularDrag = 3.5f;
}

void Rigidbody::ApplyImpulses(std::vector<SplitInfo>& splits)
{
	TransferAdditionalImpulses();

	Impulse* biggest = nullptr;
	auto biggestImpact = 0.0f;

	for  (auto it = m_Impulses.begin(); it != m_Impulses.end(); it++)
	{
		auto& imp = *it;
		auto impact = imp.Impact;

		ApplyImpulse(imp);

		if (impact > biggestImpact)
		{
			biggest = &imp;
			biggestImpact = impact;
		}
	}

	m_Impulses.clear();

	static constexpr auto impactNeededForSplit = 1.0f;

	if (biggest && (biggest->Impact > impactNeededForSplit))
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
	t.SetRotation(q + m_AngularVelocityWorld * q * 0.5f * PhysicsTime::FixedDeltaTime);
}

void Rigidbody::ApplyImpulse(const Impulse& impulse)
{
	m_VelocityWorld += impulse.WorldImpulse / GetMass();

	auto& r = impulse.WorldImpulsePoint - GetTransform().GetPosition();
	auto& J = impulse.WorldImpulse;

	m_AngularVelocityWorld += (GetInertiaInverseWorld() * Vector3::Cross(r, J));
}

void Rigidbody::CalculateForces()
{
	static constexpr float g = 9.8f;

	m_AddedForceWorld -= GetMass() * g * Vector3::Up();
	m_AddedForceWorld -= m_Drag * m_VelocityWorld;

	m_AddedMomentsWorld -= m_AngularDrag * m_AngularVelocityWorld;
}

void  Rigidbody::Integrate()
{
	m_VelocityWorld += (m_AddedForceWorld / GetMass()) * PhysicsTime::FixedDeltaTime;
	m_AngularVelocityWorld += (GetInertiaInverseWorld() * m_AddedMomentsWorld) * PhysicsTime::FixedDeltaTime;
}

void Rigidbody::ApplyNormalForces()
{
	CalculateForces();
	Integrate();

	m_AddedForceWorld = Vector3::Zero();
	m_AddedMomentsWorld = Vector3::Zero();
}

void Rigidbody::Update(std::vector<SplitInfo>& splits)
{
	ApplyNormalForces();
	ApplyImpulses(splits);
	UpdateTransform();
}