#include "pch.h"
#include "DynamicMesh.h"
#include "World.h"

std::vector<Vertex> DynamicMesh::m_VertexMemory;
std::vector<unsigned short> DynamicMesh::m_IndexMemory;

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
	SetIndexCount(m_IndexMemory.size());
	GetWorld().GetRenderer().InsertIntoBuffer(m_IndexBuffer.get(), m_IndexMemory.data(), m_IndexMemory.size());
}

void DynamicMesh::UnMapVertexBuffer()
{
	LazyRegister();
	SetVertCount(m_VertexMemory.size());
	GetWorld().GetRenderer().InsertIntoBuffer(m_VertexBuffer.get(), m_VertexMemory.data(), m_VertexMemory.size());
}

void DynamicMesh::SetVertCount(int count)
{
	auto& b = GetWorld().GetRenderer().GetBuffers();

	if (m_VertexBuffer.get())
		b.ReturnVertexBuffer(std::move(m_VertexBuffer));

	m_VertexBuffer = b.GetVertexBuffer(count);
}

void DynamicMesh::SetIndexCount(int count)
{
	auto& b = GetWorld().GetRenderer().GetBuffers();

	if (m_IndexBuffer.get())
		b.ReturnIndexBuffer(std::move(m_IndexBuffer));

	m_IndexBuffer = b.GetIndexBuffer(count);
	m_CurrIndexCount = count;
}