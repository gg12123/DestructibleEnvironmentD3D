#include "pch.h"
#include "ShapeProxy.h"
#include "Shape.h"
#include "World.h"

void ShapeProxy::Awake()
{
	if (!m_Shape)
	{
		m_Shape = &GetWorld().GetPhysics().AddDynamicRigidbody(*this);
	}
}

void ShapeProxy::Syncronise()
{
	// called on game thread whilst physics is doing collision detection.
	// At this time, the body state is mostly read only so it is safe to copy
	// the body transform into this transform, and re-mesh the shape if required
	if (m_Shape->IsDirty())
	{
		SetVertCount(m_Shape->GetRequiredVertexCount());
		SetIndexCount(m_Shape->GetRequiredIndexCount());

		auto verts = MapVertexBuffer();
		auto indicies = MapIndexBuffer();

		// fill the buffers

		UnMapVertexBuffer();
		UnMapIndexBuffer();

		m_Shape->ClearDirty();
	}

	GetTransform().SetEqualTo(m_Shape->GetTransform());
}