#pragma once
#include <vector>
#include <memory>
#include "Common\DeviceResources.h"

class IMeshRenderer;
class Light;
class Camera;
class Transform;

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

	void BindVertexBuffer(ID3D11Buffer* buffer, unsigned int stride, unsigned int offset);
	void BindIndexBuffer(ID3D11Buffer* buffer);
	void SetObjectToWorld(const Transform& transform);
	void Draw(int indexCount);

private:
	std::shared_ptr<DX::DeviceResources> m_DeviceResources;
	ID3D11DeviceContext3* m_Context;
	std::unique_ptr<ID3D11Buffer> m_ObjectToWorldConstantBuffer;

	std::vector<IMeshRenderer*> m_Renderers;

	// buffer pools here

	Camera* m_Camera;
	Light* m_Light;
};
