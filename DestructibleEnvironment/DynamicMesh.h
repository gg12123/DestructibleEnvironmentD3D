#pragma once
#include "IMeshRenderer.h"
#include "Entity.h"
#include "Vertex.h"
#include "D3DBuffers.h"
#include "ArrayWrapper.h"
#include "Constants.h"

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
		m_VertexMemory.Clear();
		return m_VertexMemory;
	}

	auto& MapIndexBuffer()
	{
		m_IndexMemory.Clear();
		return m_IndexMemory;
	}

	void UnMapVertexBuffer();
	void UnMapIndexBuffer();

private:
	void LazyRegister();

	static ArrayWrapper<Vertex, Constants::MaxNumVerts> m_VertexMemory;
	static ArrayWrapper<unsigned short, Constants::MaxNumIndicies> m_IndexMemory;

	int m_CurrVertCount = 0;
	int m_CurrIndexCount = 0;

	D3DBuffers::BufferPtr m_VertexBuffer = D3DBuffers::NullBufferPtr();
	D3DBuffers::BufferPtr m_IndexBuffer = D3DBuffers::NullBufferPtr();

	bool m_Registerd = false;
};