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

	template<class T>
	T* As()
	{
		return dynamic_cast<T*>(this);
	}

	void* operator new(size_t i)
	{
		return _mm_malloc(i, 16);
	}

	void operator delete(void* p)
	{
		_mm_free(p);
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