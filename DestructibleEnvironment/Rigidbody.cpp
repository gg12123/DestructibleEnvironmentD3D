#include "pch.h"
#include "Rigidbody.h"
#include "PhysicsTime.h"
#include "MathU.h"
#include "Shape.h"

void Rigidbody::CalculateInertia()
{
	//auto m = GetMass();
	//
	//auto Ixx = 0.0f;
	//auto Iyy = 0.0f;
	//auto Izz = 0.0f;
	//
	//auto Ixy = 0.0f;
	//auto Ixz = 0.0f;
	//auto Iyz = 0.0f;
	//
	//for (auto s : GetSubShapes())
	//{
	//	auto& points = s->GetCachedPoints();
	//
	//	for (auto it = points.begin(); it != points.end(); it++)
	//	{
	//		auto P = (*it);
	//
	//		Ixx += (P.y * P.y + P.z * P.z) * m;
	//		Iyy += (P.x * P.x + P.z * P.z) * m;
	//		Izz += (P.x * P.x + P.y * P.y) * m;
	//
	//		Ixy += (P.x * P.y) * m;
	//		Ixz += (P.x * P.z) * m;
	//		Iyz += (P.y * P.z) * m;
	//	}
	//}
	//
	//Matrix3 inertia;
	//
	//auto col0 = inertia.M[0];
	//col0[0] = Ixx;
	//col0[1] = -Ixy;
	//col0[2] = -Ixz;
	//
	//auto col1 = inertia.M[1];
	//col1[0] = -Ixy;
	//col1[1] = Iyy;
	//col1[2] = -Iyz;
	//
	//auto col2 = inertia.M[2];
	//col2[0] = -Ixz;
	//col2[1] = -Iyz;
	//col2[2] = Izz;
	//
	////SetInertia(inertia);

	// TODO
	SetInertia(Matrix3::Indentity());
}

void Rigidbody::CalculateMassProperties()
{
	SetMass(1.0f); // TODO
	CalculateInertia();
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