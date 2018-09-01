#include "pch.h"
#include "GameControlledDynamicBody.h"
#include "Rigidbody.h"
#include "World.h"

Shape & GameControlledDynamicBody::RegisterWithPhysics()
{
	m_Rigidbody = &GetWorld().GetPhysics().AddGameControlledRigidbody(*this);
	return *m_Rigidbody;
}

void GameControlledDynamicBody::AddForce(const Vector3& force)
{
	m_Rigidbody->AddForce(force);
}

void GameControlledDynamicBody::AddTorque(const Vector3& torque)
{
	m_Rigidbody->AddMoment(torque);
}