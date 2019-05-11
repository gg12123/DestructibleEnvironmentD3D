#pragma once
#include "IMeshRenderer.h"
#include "Entity.h"
#include "Vertex.h"
#include "D3DBuffers.h"
#include "Constants.h"

class DynamicMesh : public Entity, IMeshRenderer
{
public:
	void Render(Renderer& renderer) override;

	void SetColour(const Vector3& col)
	{
		m_Colour = col;
	}

	const auto& GetColour() const
	{
		return m_Colour;
	}

protected:
	auto& MapVertexBuffer()
	{
		m_VertexMemory.clear();
		return m_VertexMemory;
	}

	auto& MapIndexBuffer()
	{
		m_IndexMemory.clear();
		return m_IndexMemory;
	}

	void UnMapVertexBuffer();
	void UnMapIndexBuffer();

private:
	void SetVertCount(int count);
	void SetIndexCount(int count);

	void LazyRegister();

	static std::vector<Vertex> m_VertexMemory;
	static std::vector<unsigned short> m_IndexMemory;

	D3DBuffers::BufferPtr m_VertexBuffer = D3DBuffers::NullBufferPtr();
	D3DBuffers::BufferPtr m_IndexBuffer = D3DBuffers::NullBufferPtr();

	int m_CurrIndexCount;

	bool m_Registerd = false;
	Vector3 m_Colour = Vector3(1.0f, 0.0f, 0.0f);
};