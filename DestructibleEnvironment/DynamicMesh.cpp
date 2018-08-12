#include "pch.h"
#include "DynamicMesh.h"
#include "World.h"

void DynamicMesh::Render(Renderer& renderer)
{
	renderer.BindVertexBuffer(m_VertexBuffer, sizeof(Vertex), 0U);
	renderer.BindIndexBuffer(m_IndexBuffer);
	renderer.SetObjectToWorld(GetTransform());

	renderer.Draw(m_CurrIndexCount);
}

void DynamicMesh::UnMapVertexBuffer()
{
	if (!m_Registerd)
	{
		GetWorld().GetRenderer().Register(*this);
		m_Registerd = true;
	}
}