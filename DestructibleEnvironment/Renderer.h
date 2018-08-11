#pragma once
#include <vector>

class IMeshRenderer;
class Light;
class Camera;

class Renderer
{
public:
	void Register(IMeshRenderer& toReg)
	{
		m_Renderers.push_back(&toReg);
	}

	void AddLight(Light& light)
	{
		m_Light = &light;
	}

	void SetCamera(Camera& cam)
	{
		m_Camera = &cam;
	}

	void Render();

private:
	std::vector<IMeshRenderer*> m_Renderers;

	Camera* m_Camera;
	Light* m_Light;
};
