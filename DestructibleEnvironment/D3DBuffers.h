#pragma once
#include <memory>
#include "Common\DeviceResources.h"
#include "Pool.h"
#include "Vertex.h"
#include "Common\DirectXHelper.h"

auto D3DBufferDeleter = [](ID3D11Buffer* buffer)
{
	buffer->Release();
	delete buffer;
};

class D3DBuffers
{
public:
	using BufferPtr = std::unique_ptr<ID3D11Buffer, decltype(D3DBufferDeleter)>;

	static BufferPtr NullBufferPtr()
	{
		return BufferPtr(nullptr, D3DBufferDeleter);
	}

	static ID3D11Buffer* CreateDynamicIndexBuffer(int size, ID3D11Device3* device)
	{
		ID3D11Buffer* buffer;

		CD3D11_BUFFER_DESC indexBufferDesc(size * sizeof(unsigned short), D3D11_BIND_INDEX_BUFFER, D3D11_USAGE_DYNAMIC);
		DX::ThrowIfFailed(
			device->CreateBuffer(
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
	static BufferPtr CreateConstantBuffer(ID3D11Device3* device)
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

		return BufferPtr(buffer, D3DBufferDeleter);
	}

	D3DBuffers(ID3D11Device3* device)
	{
		auto vertBuffCreator = [=]()
		{
			return BufferPtr(CreateDynamicVertexBuffer<Vertex>(VertexBufferSize, device), D3DBufferDeleter);
		};

		auto indexBuffCreator = [=]()
		{
			return BufferPtr(CreateDynamicIndexBuffer(VertexBufferSize, device), D3DBufferDeleter);
		};

		const auto intialPoolSize = 100;

		m_VertexBufferPool = std::unique_ptr<Pool<BufferPtr>>(new Pool<BufferPtr>(vertBuffCreator, intialPoolSize));
		m_IndexBufferPool = std::unique_ptr<Pool<BufferPtr>>(new Pool<BufferPtr>(indexBuffCreator, intialPoolSize));
		m_Device = device;
	}

	BufferPtr GetVertexBuffer(int reqSize)
	{
		if (reqSize <= VertexBufferSize)
			return m_VertexBufferPool->GetObject();

		return BufferPtr(CreateDynamicVertexBuffer<Vertex>(reqSize, m_Device), D3DBufferDeleter);
	}

	BufferPtr GetIndexBuffer(int reqSize)
	{
		if (reqSize <= IndexBufferSize)
			return m_IndexBufferPool->GetObject();

		return BufferPtr(CreateDynamicIndexBuffer(reqSize, m_Device), D3DBufferDeleter);
	}

	void ReturnIndexBuffer(const BufferPtr& buf)
	{
		m_IndexBufferPool->Return(buf);
	}

	void ReturnVertexBuffer(const BufferPtr& buf)
	{
		m_VertexBufferPool->Return(buf);
	}

private:
	static constexpr int VertexBufferSize = 100;
	static constexpr int IndexBufferSize = 100;

	std::unique_ptr<Pool<BufferPtr>> m_VertexBufferPool;
	std::unique_ptr<Pool<BufferPtr>> m_IndexBufferPool;

	ID3D11Device3* m_Device;
};
