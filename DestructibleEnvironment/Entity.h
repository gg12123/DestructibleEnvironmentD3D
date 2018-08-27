#pragma once
#include "Transform.h"

class World;

class Entity
{
public:

	virtual ~Entity()
	{
	}

	void Awake(World& world)
	{
		m_World = &world;
		Awake();
	}

	Transform & GetTransform()
	{
		return m_Transform;
	}

	virtual void Update()
	{
	}

protected:

	World & GetWorld()
	{
		return *m_World;
	}

	virtual void Awake()
	{
	}

private:
	Transform m_Transform;
	World* m_World;
};