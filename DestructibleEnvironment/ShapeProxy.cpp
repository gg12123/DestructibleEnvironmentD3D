#include "pch.h"
#include "ShapeProxy.h"
#include "Shape.h"
#include "World.h"

ShapeProxy::ShapeProxy(Shape& shape)
{
	m_Shape = &shape;
}

void ShapeProxy::Awake()
{
	if (!m_Shape)
	{
		m_Shape = &RegisterWithPhysics(); // GetWorld().GetPhysics().AddDynamicRigidbody(*this);
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

		auto& verts = MapVertexBuffer();
		auto& indicies = MapIndexBuffer();

		// fill the buffers
		for (auto it = m_Shape->GetFaces().begin(); it != m_Shape->GetFaces().end(); it++)
		{
			auto f = *it;
			auto& points = f->GetCachedPoints();
			auto normal = f->GetNormal();
			auto baseIndex = verts.GetCurrCount();
			auto numPoints = points.size();
			
			verts.Add(Vertex(points.at(0), normal));

			for (auto i = 1U; i < numPoints - 1U; i++)
			{
				verts.Add(Vertex(points.at(i), normal));

				indicies.Add(baseIndex);
				indicies.Add(baseIndex + i);
				indicies.Add(baseIndex + i + 1U);
			}

			verts.Add(Vertex(points.at(numPoints - 1), normal));
		}

		UnMapVertexBuffer();
		UnMapIndexBuffer();

		m_Shape->ClearDirty();
	}

	GetTransform().SetEqualTo(m_Shape->GetTransform());
}