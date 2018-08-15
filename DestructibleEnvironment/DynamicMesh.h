#pragma once
#include "IMeshRenderer.h"
#include "Entity.h"
#include "Vertex.h"
#include "D3DBuffers.h"

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
	void SetVertCount(int count)
	{
		if (count > m_MaxVertCount)
		{
			// grab a new buffer from a pool
		}

		m_CurrVertCount = count;
	}

	void SetIndexCount(int count)
	{
		if (count > m_MaxIndexCount)
		{
			// grab a new buffer from a pool
		}

		m_CurrIndexCount = count;
	}

	Vertex* MapVertexBuffer();
	void UnMapVertexBuffer();

	int* MapIndexBuffer();
	void UnMapIndexBuffer();

private:
	int m_CurrVertCount = 0;
	int m_MaxVertCount = 0;

	int m_CurrIndexCount = 0;
	int m_MaxIndexCount = 0;

	D3DBuffers::BufferPtr m_VertexBuffer = D3DBuffers::NullBufferPtr();
	D3DBuffers::BufferPtr m_IndexBuffer = D3DBuffers::NullBufferPtr();

	bool m_Registerd = false;
};