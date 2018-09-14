#include "pch.h"
#include "Physics.h"
#include "PhysicsEngine.h"
#include "Shape.h"
#include "ShapeProxy.h"
#include "StaticShapeProxy.h"
#include "DynamicBodyProxy.h"
#include "InitialShapeCreator.h"
#include "World.h"
#include "StaticBody.h"
#include "GameControlledDynamicBody.h"

Shape & Physics::AddStaticRigidbody(StaticShapeProxy& proxy)
{
	auto body = std::unique_ptr<StaticBody>(new StaticBody()); // pool
	InitialShapeCreator::Create(*body, proxy.GetInitialWidth(), proxy.GetInitialHeight(), proxy.GetTransform());

	auto& toRet = *body;

	m_GameToPhysicsActions.emplace_back(std::unique_ptr<IGameTheadToPhysicsThreadAction>(new AddStaticRigidbodyAction(std::move(body))));

	// this proxy has come from the world so just push it onto the list
	m_ShapeProxies.push_back(&proxy);

	return toRet;
}

Rigidbody & Physics::AddDynamicRigidbody(DynamicBodyProxy& proxy)
{
	auto body = std::unique_ptr<Rigidbody>(new Rigidbody()); // pool
	InitialShapeCreator::Create(*body, proxy.GetInitialWidth(), proxy.GetInitialHeight(), proxy.GetTransform());

	auto& b = *body;

	b.CalculateMotionProperties();

	m_GameToPhysicsActions.emplace_back(std::unique_ptr<IGameTheadToPhysicsThreadAction>(new AddDynamicRigidbodyAction(std::move(body))));

	// this proxy has come from the world so just push it onto the list
	m_ShapeProxies.push_back(&proxy);

	return b;
}

Rigidbody & Physics::AddGameControlledRigidbody(GameControlledDynamicBody& proxy)
{
	auto& body = AddDynamicRigidbody(proxy);
	m_GameControlledProxies.emplace_back(&proxy);

	body.SetMass(proxy.GetMass());
	body.SetDrag(proxy.GetDrag());
	body.SetAngularDrag(proxy.GetAngularDrag());

	return body;
}

void Physics::Syncronise()
{
	if (m_Engine.IsSafeToSync())
	{
		// physics engine is doing collision detection. This is when it is safe to sync state

		// trasfer actions into the physics engine. It will then execute these
		// when it gets to updating the bodies - after this sync phase
		m_Engine.GetGameToPhysicsActions().swap(m_GameToPhysicsActions);
		m_GameToPhysicsActions.clear();

		// create proxies for any bodies that were added by the engine during its last updateBodies() step.
		CreateProxiesForBodiesAddedByEngine();

		// tell the proxies to sync.
		for (auto it = m_ShapeProxies.begin(); it != m_ShapeProxies.end(); it++)
			(*it)->Syncronise();

		// allow game to add forces etc to physics objects
		for (auto it = m_GameControlledProxies.begin(); it != m_GameControlledProxies.end(); it++)
			(*it)->FixedUpdate();

		m_Engine.ClearSafeToSync();
	}
}

void Physics::CreateProxiesForBodiesAddedByEngine()
{
	auto& newBodies = m_Engine.GetBodiesAdded();
	for (auto it = newBodies.begin(); it != newBodies.end(); it++)
		CreateShapeProxyForBodyAddedByPhysics(**it);

	newBodies.clear();
}

void Physics::CreateShapeProxyForBodyAddedByPhysics(Shape& shape)
{
	auto prox = new DynamicBodyProxy(shape);

	// this proxy has been created for a shape that was added by the physics thread
	// so it needs registering with the world
	m_World->RegisterEntity(*prox);

	m_ShapeProxies.push_back(prox);
}