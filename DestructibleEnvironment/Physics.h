#pragma once
#include "Vector.h"
#include <vector>

class Shape;
class ShapeProxy;
class PhysicsEngine;

class Physics
{
public:
	Shape & AddDynamicRigidbody(ShapeProxy& proxy);

	void Syncronise();

private:
	void CreateShapeProxy(Shape& shape);
	void CreateProxiesForBodiesAddedByEngine();

	std::vector<Shape*> m_AddedByGameThread;
	std::vector<ShapeProxy*> m_ShapeProxies;

	PhysicsEngine* m_Engine; // make unique
};
