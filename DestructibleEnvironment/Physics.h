#pragma once
#include <vector>
#include "Vector3.h"
#include "PhysicsEngine.h"
#include "InitialShapeCreator.h"
#include "RayCasting.h"

class Shape;
class ShapeProxy;
class StaticShapeProxy;
class DynamicBodyProxy;
class PhysicsEngine;
class World;

class Physics
{
public:
	Rigidbody & AddDynamicRigidbody(DynamicBodyProxy& proxy);
	StaticBody & AddStaticRigidbody(StaticShapeProxy& proxy);
	void AddJoint(const Joint& j)
	{
		m_Engine.AddJoint(j);
	}

	void Syncronise();
	void TickPhysicsEngine();

	void SetWorld(World& w)
	{
		m_World = &w;
	}

	RayCastHit<ShapeProxy> RayCast(const Ray& r) const;

	void DestructBody(DynamicBodyProxy& body, const Impulse& casue);

private:
	void CreateShapeProxyForBodyAddedByDestruct(Rigidbody& shape);
	void AddNewProxy(ShapeProxy& proxy, CompoundShape& physicsShape);

	std::vector<ShapeProxy*> m_ShapeProxies; // All the proxies

	PhysicsEngine m_Engine;
	InitialShapeCreator m_ShapeCreator;

	World* m_World;
};
