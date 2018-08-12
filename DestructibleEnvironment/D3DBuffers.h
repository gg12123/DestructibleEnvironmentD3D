#pragma once
#include "Common\DeviceResources.h"

class D3DBuffers
{
public:

	template<class T>
	static ID3D11Buffer* CreateDynamicIndexBuffer(int size, ID3D11Device3* device)
	{
		ID3D11Buffer* buffer;

		CD3D11_BUFFER_DESC indexBufferDesc(size * sizeof(unsigned short), D3D11_BIND_INDEX_BUFFER, D3D11_USAGE_DYNAMIC);
		DX::ThrowIfFailed(
			m_deviceResources->GetD3DDevice()->CreateBuffer(
				&indexBufferDesc,
				nullptr,
				&buffer
			)
		);

		return buffer;
	}

	template<class T>
	static ID3D11Buffer* CreateDynamicVertexBuffer(int size, ID3D11Device3* device)
	{
		ID3D11Buffer* buffer;

		CD3D11_BUFFER_DESC vertexBufferDesc(size * sizeof(T), D3D11_BIND_VERTEX_BUFFER, D3D11_USAGE_DYNAMIC);
		DX::ThrowIfFailed(
			device->CreateBuffer(
				&vertexBufferDesc,
				nullptr,
				&buffer
			)
		);

		return buffer;
	}

	template<class T>
	static ID3D11Buffer* CreateConstantBuffer(ID3D11Device3* device)
	{
		ID3D11Buffer* buffer;

		CD3D11_BUFFER_DESC constantBufferDesc(sizeof(T), D3D11_BIND_CONSTANT_BUFFER);
		DX::ThrowIfFailed(
			device->CreateBuffer(
				&constantBufferDesc,
				nullptr,
				&buffer
			)
		);

		return buffer;
	}
};
