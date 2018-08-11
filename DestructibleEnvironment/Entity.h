#pragma once
#include "Transform.h"

class World;

class Entity
{
public:

	void Awake(World& world)
	{
		m_World = &world;
		Awake();
	}

	Transform & GetTransform()
	{
		return m_Transform;
	}

protected:

	World & GetWorld();

	virtual void Awake()
	{
	}

private:
	Transform m_Transform;
	World* m_World;
};