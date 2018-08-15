#pragma once
#include <vector>
#include <memory>
#include "Common\DeviceResources.h"
#include "ConstantBufferInputTypes.h"
#include "D3DBuffers.h"

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

	auto& GetBuffers()
	{
		return *m_Buffers;
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

	D3DBuffers::BufferPtr m_PerObjectConstantBuffer = D3DBuffers::NullBufferPtr();
	D3DBuffers::BufferPtr m_PerSceneConstantBuffer = D3DBuffers::NullBufferPtr();

	PerObjectShaderConstants m_PerObjectConstants;
	PerSceneShaderConstants m_PerSceneConstants;

	std::vector<IMeshRenderer*> m_Renderers;

	ID3D11VertexShader* m_VertexShader;
	ID3D11PixelShader* m_PixelShader;

	ID3D11InputLayout*	m_InputLayout;

	std::unique_ptr<D3DBuffers> m_Buffers;

	Camera* m_Camera;
	Light* m_Light;
};
