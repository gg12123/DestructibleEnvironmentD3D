#pragma once
#include "IMeshRenderer.h"
#include "Entity.h"
#include "Vertex.h"
#include "D3DBuffers.h"
#include "ArrayWrapper.h"

class DynamicMesh : public Entity, IMeshRenderer
{
public:
	void Render(Renderer& renderer) override;

	int GetVertCount() const
	{
		return m_CurrVertCount;
	}

	int GetIndexCount() const
	{
		return m_CurrVertCount;
	}

protected:
	void SetVertCount(int count);
	void SetIndexCount(int count);

	auto& MapVertexBuffer()
	{
		return m_VertexMemory;
	}

	auto& MapIndexBuffer()
	{
		return m_IndexMemory;
	}

	void UnMapVertexBuffer();
	void UnMapIndexBuffer();

private:
	void LazyRegister();

	static constexpr int MaxVertCount = 300;
	static constexpr int MaxIndexCount = 300;

	static ArrayWrapper<Vertex, MaxVertCount> m_VertexMemory;
	static ArrayWrapper<unsigned short, MaxIndexCount> m_IndexMemory;

	int m_CurrVertCount = 0;
	int m_CurrIndexCount = 0;

	D3DBuffers::BufferPtr m_VertexBuffer = D3DBuffers::NullBufferPtr();
	D3DBuffers::BufferPtr m_IndexBuffer = D3DBuffers::NullBufferPtr();

	bool m_Registerd = false;
};