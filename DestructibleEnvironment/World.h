#pragma once
#include "Renderer.h"
#include "Physics.h"

class World
{
public:
	Renderer & GetRenderer()
	{
		return m_Renderer;
	}

	Physics& GetPhysics()
	{
		return m_Physics;
	}

private:
	Renderer m_Renderer;
	Physics m_Physics;
};
