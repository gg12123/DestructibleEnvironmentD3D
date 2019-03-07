#include "pch.h"
#include "DynamicBodyProxy.h"
#include "World.h"
#include "Rigidbody.h"

DynamicBodyProxy::DynamicBodyProxy(Rigidbody& body) : ShapeProxy(body)
{
	SetRigidBody(body);
}

CompoundShape & DynamicBodyProxy::RegisterWithPhysics()
{
	auto& b = GetWorld().GetPhysics().AddDynamicRigidbody(*this);
	SetRigidBody(b);
	return b;
}

void DynamicBodyProxy::AddForce(const Vector3& force)
{
	assert(Physics::AcceptingGameInput());
	m_Body->AddForce(force);
}

void DynamicBodyProxy::AddTorque(const Vector3& torque)
{
	assert(Physics::AcceptingGameInput());
	m_Body->AddMoment(torque);
}

void DynamicBodyProxy::AddImpulse(const Impulse& imp)
{
	assert(Physics::AcceptingGameInput());
	m_Body->AddExternalImpulse(imp);
}