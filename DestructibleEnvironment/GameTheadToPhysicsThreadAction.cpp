#include "GameThreadToPhysicsThreadAction.h"
#include "PhysicsEngine.h"

void AddDynamicRigidbodyAction::Apply(PhysicsEngine& engine)
{
	engine.GetDynamicBodies().emplace_back(std::move(m_BodyToAdd));
}

void AddStaticRigidbodyAction::Apply(PhysicsEngine& engine)
{
	engine.GetStaticBodies().emplace_back(std::move(m_BodyToAdd));
}