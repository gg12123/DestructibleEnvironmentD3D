#include "pch.h"
#include "Renderer.h"
#include "IMeshRenderer.h"
#include "D3DBuffers.h"
#include "Transform.h"
#include "Camera.h"
#include "Light.h"
#include "Common\DirectXHelper.h"

Renderer::Renderer(const std::shared_ptr<DX::DeviceResources>& deviceResources) : m_DeviceResources(deviceResources)
{
	m_Context = m_DeviceResources->GetD3DDeviceContext();

	m_PerObjectConstantBuffer = D3DBuffers::CreateConstantBuffer<PerObjectShaderConstants>(m_DeviceResources->GetD3DDevice());
	m_PerSceneConstantBuffer = D3DBuffers::CreateConstantBuffer<PerSceneShaderConstants>(m_DeviceResources->GetD3DDevice());

	CreateShaders();
}

void Renderer::CreateShaders()
{
	auto loadVSTask = DX::ReadDataAsync(L"MyVertexShader.cso");
	auto loadPSTask = DX::ReadDataAsync(L"MyPixelShader.cso");

	loadVSTask.wait();
	loadPSTask.wait();

	auto& vsFileData = loadVSTask.get();
	auto& psFileData = loadPSTask.get();

	DX::ThrowIfFailed(m_DeviceResources->GetD3DDevice()->CreateVertexShader( &vsFileData[0], vsFileData.size(),
			nullptr, &m_VertexShader ) );

	DX::ThrowIfFailed(m_DeviceResources->GetD3DDevice()->CreatePixelShader(&psFileData[0], psFileData.size(),
			nullptr, &m_PixelShader));

	static const D3D11_INPUT_ELEMENT_DESC vertexDesc[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	};

	DX::ThrowIfFailed(m_DeviceResources->GetD3DDevice()->CreateInputLayout(vertexDesc, ARRAYSIZE(vertexDesc),
			&vsFileData[0], vsFileData.size(), &m_InputLayout));
}

void Renderer::BindVertexBuffer(ID3D11Buffer* buffer, unsigned int stride, unsigned int offset)
{
	m_Context->IASetVertexBuffers(0, 1, &buffer, &stride, &offset);
}

void Renderer::BindIndexBuffer(ID3D11Buffer* buffer)
{
	m_Context->IASetIndexBuffer(buffer, DXGI_FORMAT_R32_UINT, 0);
}

void Renderer::SetObjectToWorld(const Transform& transform)
{
	m_PerObjectConstants.ModelToWorldMatrix = transform.GetLocalToWorldMatrix();

	m_Context->UpdateSubresource1(m_PerObjectConstantBuffer, 0, NULL,
		&m_PerObjectConstants,
		0, 0, 0);
}

void Renderer::Draw(int indexCount)
{
	m_Context->DrawIndexed(indexCount, 0, 0);
}

void Renderer::Render()
{
	m_Context->VSSetShader(m_VertexShader, nullptr, 0);
	m_Context->PSSetShader(m_PixelShader, nullptr, 0);

	m_Context->VSSetConstantBuffers1(PerObjectConstBufferSlot, 1, &m_PerObjectConstantBuffer, nullptr, nullptr);
	m_Context->VSSetConstantBuffers1(PerSceneConstBufferSlot, 1, &m_PerSceneConstantBuffer, nullptr, nullptr);

	m_PerSceneConstants.VPMatrix = m_Camera->CalcuateVPMatrix();
	m_PerSceneConstants.LightPosition = m_Light->GetLightPosition();
	m_PerSceneConstants.ViewDirection = m_Camera->GetViewDirection();

	m_Context->UpdateSubresource1(m_PerSceneConstantBuffer, 0, NULL,
		&m_PerSceneConstants,
		0, 0, 0);

	m_Context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	m_Context->IASetInputLayout(m_InputLayout);

	for (auto it = m_Renderers.begin(); it != m_Renderers.end(); it++)
		(*it)->Render(*this);
}