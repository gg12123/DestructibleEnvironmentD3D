#include "pch.h"
#include "Physics.h"
#include "PhysicsEngine.h"
#include "Shape.h"
#include "ShapeProxy.h"

Shape & Physics::AddDynamicRigidbody(ShapeProxy& proxy)
{
	auto shape = new Shape(); // pool
	m_BodiesToBeAdded.push_back(shape);

	CreateShapeProxy(*shape);

	return *shape;
}

void Physics::Syncronise()
{
	if (m_Engine->IsSafeToSync())
	{
		// physics engine is doing collision detection. This is when it is safe to sync state

		// trasfer the bodies added by the game thread into the physics engine. It will then transfer these
		// into its main collection when it gets to updating the bodies - after this sync phase
		m_Engine->GetBodiesToBeAdded().swap(m_BodiesToBeAdded);
		m_BodiesToBeAdded.clear();

		// create proxies for any bodies that were added by the engine during its last updateBodies() step.
		CreateProxiesForBodiesAddedByEngine();

		// tell the proxies to sync.
		for (auto it = m_ShapeProxies.begin(); it != m_ShapeProxies.end(); it++)
			(*it)->Syncronise();

		m_Engine->ClearSafeToSync();
	}
}

void Physics::CreateProxiesForBodiesAddedByEngine()
{
	auto& newBodies = m_Engine->GetBodiesAdded();
	for (auto it = newBodies.begin(); it != newBodies.end(); it++)
		CreateShapeProxy(**it);

	newBodies.clear();
}