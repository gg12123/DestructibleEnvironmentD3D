#pragma once
#include <vector>
#include <memory>
#include <atomic>
#include "Common\DeviceResources.h"
#include "ConstantBufferInputTypes.h"
#include "D3DBuffers.h"
#include "ViewportDimensions.h"

class IMeshRenderer;
class Light;
class Camera;
class Transform;

class Renderer
{
public:
	Renderer()
	{
	}

	void SetResources(const std::shared_ptr<DX::DeviceResources>& deviceResources,
					const ViewportDimensions& viewDims);

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

	auto& GetActiveCamera() const
	{
		return *m_Camera;
	}

	auto& GetBuffers()
	{
		return *m_Buffers;
	}

	bool IsReadyToRender() const
	{
		return m_LoadedVS && m_LoadedPS;
	}

	auto& GetViewportDimensions() const
	{
		return m_ViewportDims;
	}

	void Render();

	void BindVertexBuffer(ID3D11Buffer* buffer, unsigned int stride, unsigned int offset);
	void BindIndexBuffer(ID3D11Buffer* buffer);
	void SetObjectToWorldAndColour(const Transform& transform, const Vector3& col);
	void Draw(int indexCount);

	template<class T>
	void InsertIntoBuffer(ID3D11Buffer* buffer, T* data, int numElements)
	{
		D3D11_MAPPED_SUBRESOURCE mappedResource;
		ZeroMemory(&mappedResource, sizeof(D3D11_MAPPED_SUBRESOURCE));

		DX::ThrowIfFailed(m_Context->Map(buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource));
		memcpy(mappedResource.pData, data, numElements * sizeof(T));
		m_Context->Unmap(buffer, 0);
	}

private:
	void CreateShaders();

	static int constexpr PerObjectConstBufferSlot = 0;
	static int constexpr PerSceneConstBufferSlot = 1;

	ViewportDimensions m_ViewportDims;
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

	std::atomic<bool> m_LoadedVS = false;
	std::atomic<bool> m_LoadedPS = false;

	Camera* m_Camera;
	Light* m_Light;
};
