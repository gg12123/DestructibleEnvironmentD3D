#include "pch.h"
#include "GameControlledDynamicBody.h"
#include "Rigidbody.h"
#include "World.h"

Shape & GameControlledDynamicBody::RegisterWithPhysics()
{
	auto& b = GetWorld().GetPhysics().AddGameControlledRigidbody(*this);
	SetRigidBody(b);
	return b;
}