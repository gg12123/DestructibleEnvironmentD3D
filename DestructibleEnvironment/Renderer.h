#pragma once
#include <vector>
#include <memory>
#include "Common\DeviceResources.h"
#include "ConstantBufferInputTypes.h"

class IMeshRenderer;
class Light;
class Camera;
class Transform;

class Renderer
{
public:
	Renderer(const std::shared_ptr<DX::DeviceResources>& deviceResources);

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
	void CreateShaders();

	static int constexpr PerObjectConstBufferSlot = 0;
	static int constexpr PerSceneConstBufferSlot = 1;

	std::shared_ptr<DX::DeviceResources> m_DeviceResources;
	ID3D11DeviceContext3* m_Context;

	ID3D11Buffer* m_PerObjectConstantBuffer;
	ID3D11Buffer* m_PerSceneConstantBuffer;

	PerObjectShaderConstants m_PerObjectConstants;
	PerSceneShaderConstants m_PerSceneConstants;

	std::vector<IMeshRenderer*> m_Renderers;

	ID3D11VertexShader* m_VertexShader;
	ID3D11PixelShader* m_PixelShader;

	ID3D11InputLayout*	m_InputLayout;

	// buffer pools here

	Camera* m_Camera;
	Light* m_Light;
};
