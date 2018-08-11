#include "DynamicMesh.h"
#include "World.h"

void DynamicMesh::UnMapVertexBuffer()
{
	if (!m_Registerd)
	{
		GetWorld().GetRenderer().Register(*this);
		m_Registerd = true;
	}
}