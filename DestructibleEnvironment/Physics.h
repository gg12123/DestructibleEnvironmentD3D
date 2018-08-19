#pragma once
#include "Vector3.h"
#include "PhysicsEngine.h"
#include <vector>

class Shape;
class ShapeProxy;
class PhysicsEngine;
class World;

class Physics
{
public:
	Shape & AddDynamicRigidbody(ShapeProxy& proxy);

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
	void CreateShapeProxy(Shape& shape);
	void CreateProxiesForBodiesAddedByEngine();

	std::vector<Shape*> m_BodiesToBeAdded;
	std::vector<ShapeProxy*> m_ShapeProxies;

	PhysicsEngine m_Engine;

	World* m_World;
};
