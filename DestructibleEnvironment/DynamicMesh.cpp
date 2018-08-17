#include "pch.h"
#include "DynamicMesh.h"
#include "World.h"

void DynamicMesh::Render(Renderer& renderer)
{
	renderer.BindVertexBuffer(m_VertexBuffer.get(), sizeof(Vertex), 0U);
	renderer.BindIndexBuffer(m_IndexBuffer.get());
	renderer.SetObjectToWorld(GetTransform());

	renderer.Draw(m_CurrIndexCount);
}

void DynamicMesh::LazyRegister()
{
	if (!m_Registerd)
	{
		GetWorld().GetRenderer().Register(*this);
		m_Registerd = true;
	}
}

void DynamicMesh::UnMapIndexBuffer()
{
	LazyRegister();
	GetWorld().GetRenderer().InsertIntoBuffer(m_IndexBuffer.get(), m_IndexMemory, m_CurrIndexCount);
}

void DynamicMesh::UnMapVertexBuffer()
{
	LazyRegister();
	GetWorld().GetRenderer().InsertIntoBuffer(m_VertexBuffer.get(), m_VertexMemory, m_CurrVertCount);
}

void DynamicMesh::SetVertCount(int count)
{
	// TODO - only switch buffer if current one is too small

	assert(count < MaxVertCount);

	auto& b = GetWorld().GetRenderer().GetBuffers();

	if (m_VertexBuffer.get())
		b.ReturnVertexBuffer(m_VertexBuffer);

	m_VertexBuffer = b.GetVertexBuffer(count);

	m_CurrVertCount = count;
}

void DynamicMesh::SetIndexCount(int count)
{
	// TODO - only switch buffer if current one is too small

	assert(count < MaxIndexCount);

	auto& b = GetWorld().GetRenderer().GetBuffers();

	if (m_IndexBuffer.get())
		b.ReturnIndexBuffer(m_IndexBuffer);

	m_IndexBuffer = b.GetIndexBuffer(count);

	m_CurrIndexCount = count;
}