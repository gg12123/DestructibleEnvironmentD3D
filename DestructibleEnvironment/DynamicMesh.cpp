#include "pch.h"
#include "DynamicMesh.h"
#include "World.h"

ArrayWrapper<Vertex, ShapeConstants::MaxNumVerts> DynamicMesh::m_VertexMemory;
ArrayWrapper<unsigned short, ShapeConstants::MaxNumIndicies> DynamicMesh::m_IndexMemory;

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
	GetWorld().GetRenderer().InsertIntoBuffer(m_IndexBuffer.get(), m_IndexMemory.GetData(), m_CurrIndexCount);
}

void DynamicMesh::UnMapVertexBuffer()
{
	LazyRegister();
	GetWorld().GetRenderer().InsertIntoBuffer(m_VertexBuffer.get(), m_VertexMemory.GetData(), m_CurrVertCount);
}

void DynamicMesh::SetVertCount(int count)
{
	// TODO - only switch buffer if current one is too small

	assert(count < ShapeConstants::MaxNumVerts);

	auto& b = GetWorld().GetRenderer().GetBuffers();

	if (m_VertexBuffer.get())
		b.ReturnVertexBuffer(std::move(m_VertexBuffer));

	m_VertexBuffer = b.GetVertexBuffer(count);

	m_CurrVertCount = count;
}

void DynamicMesh::SetIndexCount(int count)
{
	// TODO - only switch buffer if current one is too small

	assert(count < ShapeConstants::MaxNumIndicies);

	auto& b = GetWorld().GetRenderer().GetBuffers();

	if (m_IndexBuffer.get())
		b.ReturnIndexBuffer(std::move(m_IndexBuffer));

	m_IndexBuffer = b.GetIndexBuffer(count);

	m_CurrIndexCount = count;
}