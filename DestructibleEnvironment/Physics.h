#pragma once
#include <vector>
#include "Vector3.h"
#include "PhysicsEngine.h"
#include "GameThreadToPhysicsThreadAction.h"
#include "InitialShapeCreator.h"
#include "RayCasting.h"
#include "IOnPhysicsWorldUpdated.h"

class Shape;
class ShapeProxy;
class StaticShapeProxy;
class DynamicBodyProxy;
class PhysicsEngine;
class World;
class GameControlledDynamicBody;

class Physics
{
public:
	Rigidbody & AddGameControlledRigidbody(GameControlledDynamicBody& proxy);
	Rigidbody & AddDynamicRigidbody(DynamicBodyProxy& proxy);
	CompoundShape & AddStaticRigidbody(StaticShapeProxy& proxy);

	void Syncronise();

	void SetWorld(World& w)
	{
		m_World = &w;
	}

	void StartRunningPhysicsThread()
	{
		m_Engine.StartRunning();
	}

	void StopRunningPhysicsThread()
	{
		m_Engine.StopRunning();
	}

	RayCastHit<ShapeProxy> RayCast(const Ray& r) const;

	static bool AcceptingGameInput()
	{
		return m_AcceptGameInput;
	}

	void AddOnPhysicsUpdatedListener(IOnPhysicsWorldUpdated& listener)
	{
		m_OnPhysicsUpdatedListeners.emplace_back(&listener);
	}

private:
	void CreateShapeProxyForBodyAddedByPhysics(Rigidbody& shape);
	void CreateProxiesForBodiesAddedByEngine();
	void AddNewProxy(ShapeProxy& proxy, CompoundShape& physicsShape);

	std::vector<std::unique_ptr<IGameTheadToPhysicsThreadAction>> m_GameToPhysicsActions;

	std::vector<ShapeProxy*> m_ShapeProxies; // All the proxies
	std::vector<GameControlledDynamicBody*> m_GameControlledProxies;

	std::vector<IOnPhysicsWorldUpdated*> m_OnPhysicsUpdatedListeners;

	PhysicsEngine m_Engine;
	InitialShapeCreator m_ShapeCreator;

	World* m_World;

	static bool m_AcceptGameInput;
};
