#pragma once
#include "ByteAlignment.h"
#include "Transform.h"

class World;

class Entity : public AlignedObject16
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

	template<class T>
	T* As()
	{
		return dynamic_cast<T*>(this);
	}

protected:

	World & GetWorld() const
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