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

bool Physics::m_AcceptGameInput = false;

void Physics::AddNewProxy(ShapeProxy& proxy, CompoundShape& physicsShape)
{
	m_ShapeProxies.push_back(&proxy);
}

CompoundShape & Physics::AddStaticRigidbody(StaticShapeProxy& proxy)
{
	auto body = std::unique_ptr<StaticBody>(new StaticBody());
	m_ShapeCreator.Create(*body, proxy.GetInitialWidth(), proxy.GetInitialHeight());

	auto& toRet = *body;
	toRet.InitMassProperties(proxy.GetTransform());

	m_GameToPhysicsActions.emplace_back(std::unique_ptr<IGameTheadToPhysicsThreadAction>(new AddStaticRigidbodyAction(std::move(body))));

	AddNewProxy(proxy, toRet);

	return toRet;
}

Rigidbody & Physics::AddDynamicRigidbody(DynamicBodyProxy& proxy)
{
	auto body = std::unique_ptr<Rigidbody>(new Rigidbody());
	m_ShapeCreator.Create(*body, proxy.GetInitialWidth(), proxy.GetInitialHeight());

	auto& b = *body;

	b.SetDrag(1.0f);
	b.SetAngularDrag(0.5f);
	b.InitMassProperties(proxy.GetTransform());

	m_GameToPhysicsActions.emplace_back(std::unique_ptr<IGameTheadToPhysicsThreadAction>(new AddDynamicRigidbodyAction(std::move(body))));

	AddNewProxy(proxy, b);

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

RayCastHit<ShapeProxy> Physics::RayCast(const Ray& r) const
{
	assert(m_AcceptGameInput);

	auto hit = m_Engine.RayCast(r);
	if (hit.Hit())
	{
		auto hitShape = hit.GetHitObject();
		return RayCastHit<ShapeProxy>(&hitShape->GetProxy(), hit.GetHitPoint());
	}
	return RayCastHit<ShapeProxy>(nullptr, Vector3::Zero());
}

void Physics::Syncronise()
{
	if (m_Engine.IsSafeToSync())
	{
		// Physics engine is doing collision detection. This is when it is safe to sync state.

		// Trasfer actions into the physics engine. It will then execute these
		// when it gets to updating the bodies - after this sync phase.
		m_Engine.GetGameToPhysicsActions().swap(m_GameToPhysicsActions);
		m_GameToPhysicsActions.clear();

		// Create proxies for any bodies that were added by the engine during its last updateBodies() step.
		CreateProxiesForBodiesAddedByEngine();

		// Tell the proxies to sync.
		for (auto it = m_ShapeProxies.begin(); it != m_ShapeProxies.end(); it++)
			(*it)->Syncronise();

		// Allow game to add forces etc to physics objects.
		// This is the only time ray-casting is allowed.
		m_AcceptGameInput = true;

		for (auto it = m_GameControlledProxies.begin(); it != m_GameControlledProxies.end(); it++)
			(*it)->FixedUpdate();

		for (auto x : m_OnPhysicsUpdatedListeners)
			x->OnPhysicsWorldUpdated();

		m_AcceptGameInput = false;

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

void Physics::CreateShapeProxyForBodyAddedByPhysics(Rigidbody& body)
{
	auto prox = new DynamicBodyProxy(body);

	// this proxy has been created for a shape that was added by the physics thread
	// so it needs registering with the world
	m_World->RegisterEntity(std::unique_ptr<Entity>(prox));

	AddNewProxy(*prox, body);
}