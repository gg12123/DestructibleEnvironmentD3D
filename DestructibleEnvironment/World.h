#pragma once
#include <vector>
#include <memory>
#include "Renderer.h"
#include "Physics.h"
#include "Entity.h"

class World
{
public:
	World()
	{
	}

	~World()
	{
		m_Physics.StopRunningPhysicsThread();
	}

	void Init(const std::shared_ptr<DX::DeviceResources>& deviceResources)
	{
		m_Renderer.SetResources(deviceResources);
		m_Physics.SetWorld(*this);
		m_Physics.StartRunningPhysicsThread();
	}

	Renderer& GetRenderer()
	{
		return m_Renderer;
	}

	Physics& GetPhysics()
	{
		return m_Physics;
	}

	void RegisterEntity(std::unique_ptr<Entity>&& ent);
	void RegisterEntityForUpdate(Entity& ent);

	void Update();
	void Render();

private:
	void UpdateEntities();

	Renderer m_Renderer;
	Physics m_Physics;

	std::vector<std::unique_ptr<Entity>> m_Entities;
	std::vector<Entity*> m_UpdateableEntities;
};
