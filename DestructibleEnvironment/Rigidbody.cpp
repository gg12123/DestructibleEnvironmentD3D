#include "Rigidbody.h"
#include "PhysicsTime.h"

void Rigidbody::ApplyImpulses()
{
	for  (auto it = m_Impulses.begin(); it != m_Impulses.end(); it++)
	{
		if (ApplyImpulse(**it))
			break; // TODO - do something more clever when a split occurs
	}

	m_Impulses.clear();
}

void Rigidbody::UpdateTransform()
{
	auto& t = GetTransform();

	t.SetPosition(t.GetPosition() + m_VelocityWorld * PhysicsTime::DeltaTime());

	auto& q = t.GetRotation();

	t.SetRotation(q + q * m_AngularVelocityLocal * (0.5f * PhysicsTime::DeltaTime()));
}

bool Rigidbody::ApplyImpulse(const Impulse& impulse)
{
	m_VelocityWorld += impulse.WorldImpulse / GetMass();

	auto r = impulse.LocalCollisionPoint;
	auto J = impulse.LocalImpulse;

	m_AngularVelocityLocal += (GetInertiaInverse() * Vector3::Cross(r, J));

	if (impulse.Impact > GetMass()) // TODO - use better condition
	{
		// split
		return true;
	}
	return false;
}

void Rigidbody::Update()
{

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

void Rigidbody::Update()
{
	ApplyNormalForces();
	ApplyImpulses();
	UpdateTransform(); // will need to update transform of any shapes resulting from a split
}