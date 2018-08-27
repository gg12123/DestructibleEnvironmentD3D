#pragma once
#include "Vector3.h"
#include "PhysicsEngine.h"
#include "GameThreadToPhysicsThreadAction.h"
#include <vector>

class Shape;
class ShapeProxy;
class StaticShapeProxy;
class DynamicBodyProxy;
class PhysicsEngine;
class World;

class Physics
{
public:
	Shape & AddDynamicRigidbody(DynamicBodyProxy& proxy);
	Shape & AddStaticRigidbody(StaticShapeProxy& proxy);

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

private:
	void CreateShapeProxyForBodyAddedByPhysics(Shape& shape);
	void CreateProxiesForBodiesAddedByEngine();

	std::vector<std::unique_ptr<IGameTheadToPhysicsThreadAction>> m_GameToPhysicsActions;

	std::vector<ShapeProxy*> m_ShapeProxies;

	PhysicsEngine m_Engine;

	World* m_World;
};
