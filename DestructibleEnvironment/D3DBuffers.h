#pragma once
#include <memory>
#include "Common\DeviceResources.h"
#include "Pool.h"
#include "Vertex.h"
#include "Common\DirectXHelper.h"
#include "Constants.h"
#include "MathUtils.h"

struct D3DBufferDeleter
{
public:
	void operator()(ID3D11Buffer* buffer) const
	{ 
		buffer->Release();
	}
};

class D3DBuffers
{
public:
	using BufferPtr = std::unique_ptr<ID3D11Buffer, D3DBufferDeleter>;

	static BufferPtr NullBufferPtr()
	{
		return BufferPtr(nullptr, D3DBufferDeleter());
	}

	static ID3D11Buffer* CreateDynamicIndexBuffer(int size, ID3D11Device3* device)
	{
		ID3D11Buffer* buffer;

		D3D11_BUFFER_DESC indexBufferDesc = { 0 };
		indexBufferDesc.ByteWidth = MathUtils::RoundUp(size * sizeof(unsigned short), 16);
		indexBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
		indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
		indexBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		indexBufferDesc.MiscFlags = 0;
		indexBufferDesc.StructureByteStride = 0;

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

		D3D11_BUFFER_DESC vertexBufferDesc = { 0 };
		vertexBufferDesc.ByteWidth = MathUtils::RoundUp(size * sizeof(T), 16);
		vertexBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
		vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		vertexBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		vertexBufferDesc.MiscFlags = 0;
		vertexBufferDesc.StructureByteStride = 0;

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

		CD3D11_BUFFER_DESC constantBufferDesc(MathUtils::RoundUp(sizeof(T), 16), D3D11_BIND_CONSTANT_BUFFER);
		DX::ThrowIfFailed(
			device->CreateBuffer(
				&constantBufferDesc,
				nullptr,
				&buffer
			)
		);

		return BufferPtr(buffer, D3DBufferDeleter());
	}

	D3DBuffers(ID3D11Device3* device)
	{
		std::function<BufferPtr()> vertBuffCreator = [device]()
		{
			return BufferPtr(CreateDynamicVertexBuffer<Vertex>(VertexBufferSize, device), D3DBufferDeleter());
		};

		std::function<BufferPtr()> indexBuffCreator = [device]()
		{
			return BufferPtr(CreateDynamicIndexBuffer(VertexBufferSize, device), D3DBufferDeleter());
		};

		const auto intialPoolSize = 100;

		m_VertexBufferPool = std::unique_ptr<Pool<BufferPtr>>(new Pool<BufferPtr>(std::move(vertBuffCreator), intialPoolSize));
		m_IndexBufferPool = std::unique_ptr<Pool<BufferPtr>>(new Pool<BufferPtr>(std::move(indexBuffCreator), intialPoolSize));
		m_Device = device;
	}

	BufferPtr GetVertexBuffer(int reqSize)
	{
		if (reqSize <= VertexBufferSize)
			return m_VertexBufferPool->GetObject();

		return BufferPtr(CreateDynamicVertexBuffer<Vertex>(reqSize, m_Device), D3DBufferDeleter());
	}

	BufferPtr GetIndexBuffer(int reqSize)
	{
		if (reqSize <= IndexBufferSize)
			return m_IndexBufferPool->GetObject();

		return BufferPtr(CreateDynamicIndexBuffer(reqSize, m_Device), D3DBufferDeleter());
	}

	void ReturnIndexBuffer(BufferPtr&& buf)
	{
		m_IndexBufferPool->Return(std::move(buf));
	}

	void ReturnVertexBuffer(BufferPtr&& buf)
	{
		m_VertexBufferPool->Return(std::move(buf));
	}

private:
	static constexpr int VertexBufferSize = Constants::MaxNumVerts;
	static constexpr int IndexBufferSize = Constants::MaxNumIndicies;

	std::unique_ptr<Pool<BufferPtr>> m_VertexBufferPool;
	std::unique_ptr<Pool<BufferPtr>> m_IndexBufferPool;

	ID3D11Device3* m_Device;
};
