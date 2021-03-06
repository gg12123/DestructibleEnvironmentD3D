#pragma once
#include <vector>
#include <memory>
#include "Renderer.h"
#include "Physics.h"
#include "Entity.h"
#include "ReadOnlyInput.h"
#include "ViewportDimensions.h"
#include "FixedTimeStepTime.h"

class World
{
public:
	World()
	{
	}

	~World()
	{
	}

	void Init(const std::shared_ptr<DX::DeviceResources>& deviceResources, const ReadOnlyInput& input, const ViewportDimensions& viewDims)
	{
		m_Input = input;
		m_Renderer.SetResources(deviceResources, viewDims);
		m_Physics.SetWorld(*this);
		m_PhysicsTime.SetFixedDeltaTime(PhysicsTime::FixedDeltaTime);
		m_PhysicsTime.Start();
	}

	Renderer& GetRenderer()
	{
		return m_Renderer;
	}

	Physics& GetPhysics()
	{
		return m_Physics;
	}

	auto& GetInput() const
	{
		return m_Input;
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

	ReadOnlyInput m_Input;
	FixedTimeStepTime m_PhysicsTime;
};
