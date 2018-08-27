#include "Rigidbody.h"
#include "PhysicsTime.h"

void Rigidbody::ApplyImpulses(std::vector<SplitInfo>& splits)
{
	Impulse* biggest = nullptr;
	auto biggestImpact = 0.0f;

	for  (auto it = m_Impulses.begin(); it != m_Impulses.end(); it++)
	{
		auto imp = **it;
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

	t.SetPosition(t.GetPosition() + m_VelocityWorld * PhysicsTime::DeltaTime());

	auto& q = t.GetRotation();

	t.SetRotation(q + q * m_AngularVelocityLocal * (0.5f * PhysicsTime::DeltaTime()));
}

void Rigidbody::ApplyImpulse(const Impulse& impulse)
{
	m_VelocityWorld += impulse.WorldImpulse / GetMass();

	auto r = impulse.LocalCollisionPoint;
	auto J = impulse.LocalImpulse;

	m_AngularVelocityLocal += (GetInertiaInverse() * Vector3::Cross(r, J));
}

void Rigidbody::CalculateForces(Vector3& forcesWorld, Vector3& momentsLocal)
{
	static constexpr float g = 9.8f;

	forcesWorld -= GetMass() * g * Vector3::Up();
	forcesWorld -= m_Drag * m_VelocityWorld;

	momentsLocal -= m_AngularDrag * m_AngularVelocityLocal;
}

void  Rigidbody::Integrate(const Vector3& forcesWorld, const Vector3& momentsLocal)
{
	m_VelocityWorld += (forcesWorld / GetMass()) * PhysicsTime::DeltaTime();
	m_AngularVelocityLocal += (GetInertiaInverse() * momentsLocal) * PhysicsTime::DeltaTime();
}

void Rigidbody::ApplyNormalForces()
{
	Vector3 f, m;
	CalculateForces(f, m);
	Integrate(f, m);
}

void Rigidbody::Update(std::vector<SplitInfo>& splits)
{
	ApplyNormalForces();
	ApplyImpulses(splits);
	UpdateTransform();
}