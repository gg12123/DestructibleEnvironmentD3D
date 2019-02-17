#include "pch.h"
#include "ShapeProxy.h"
#include "Shape.h"
#include "World.h"

ShapeProxy::ShapeProxy(CompoundShape& shape)
{
	m_Shape = &shape;
	m_Shape->SetProxy(*this);
}

void ShapeProxy::Awake()
{
	if (!m_Shape)
	{
		m_Shape = &RegisterWithPhysics();
		m_Shape->SetProxy(*this);
	}
}

void ShapeProxy::Syncronise()
{
	// Called on game thread whilst physics is doing collision detection.
	// At this time, the body state is mostly read only so it is safe to copy
	// the body transform into this transform and re-mesh the shape if required.

	if (m_Shape->IsDirty())
	{
		auto& verts = MapVertexBuffer();
		auto& indicies = MapIndexBuffer();

		for (auto subShape : m_Shape->GetSubShapes())
		{
			for (auto f : subShape->GetFaces())
			{
				auto& points = f->GetCachedPoints();
				auto normal = f->GetNormal();
				auto baseIndex = verts.size();
				auto numPoints = points.size();

				verts.emplace_back(Vertex(points.at(0), normal));

				for (auto i = 1U; i < numPoints - 1U; i++)
				{
					verts.emplace_back(Vertex(points.at(i), normal));

					indicies.emplace_back(baseIndex);
					indicies.emplace_back(baseIndex + i);
					indicies.emplace_back(baseIndex + i + 1U);
				}

				verts.emplace_back(Vertex(points.at(numPoints - 1), normal));
			}
		}

		UnMapVertexBuffer();
		UnMapIndexBuffer();

		m_Shape->ClearDirty();
	}

	GetTransform().SetEqualTo(m_Shape->GetTransform());
}