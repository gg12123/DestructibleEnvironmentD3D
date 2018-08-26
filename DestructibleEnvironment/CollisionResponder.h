#pragma once
#include <memory>
#include "Vector3.h"
#include "PoolOfRecyclables.h"
#include "CollisionData.h"

class PhysicsObject;

class CollisionResponder
{
public:
	CollisionResponder()
	{
		std::function<std::unique_ptr<Impulse>()> creator = []()
		{
			return std::unique_ptr<Impulse>(new Impulse());
		};

		m_ImpulseDataPool = std::unique_ptr<PoolOfRecyclables<std::unique_ptr<Impulse>>>
			(new PoolOfRecyclables<std::unique_ptr<Impulse>>(30, creator));
	}

	void Reset()
	{
		m_ImpulseDataPool->Reset();
	}

	void CalculateResponse(const CollisionData& collData, PhysicsObject& body1, PhysicsObject& body2);

private:
	std::unique_ptr<PoolOfRecyclables<std::unique_ptr<Impulse>>> m_ImpulseDataPool;
};
