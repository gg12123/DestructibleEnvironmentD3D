#include "pch.h"
#include "World.h"

void World::RegisterEntity(std::unique_ptr<Entity>&& ent)
{
	ent->Awake(*this);
	m_Entities.emplace_back(std::move(ent));
}

void World::Update()
{
	m_Physics.Syncronise();
	UpdateEntities();
}

void World::Render()
{
	m_Renderer.Render();
}

void World::UpdateEntities()
{
	for (auto it = m_UpdateableEntities.begin(); it != m_UpdateableEntities.end(); it++)
		(*it)->Update();
}

void World::RegisterEntityForUpdate(Entity& ent)
{
	m_UpdateableEntities.push_back(&ent);
}