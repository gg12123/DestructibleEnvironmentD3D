#pragma once
#include "Shape.h"
#include "ShapeEdge.h"
#include "ShapePoint.h"
#include "DynamicArray.h"
#include "Face.h"
#include "ShapeElementPool.h"

class ShapeDuplicater
{
private:
	ShapePoint & GetNewPoint(ShapePoint& existing)
	{
		if (!existing.HashIsAssigned())
		{
			existing.AssignHash();
			m_ToPoints[existing.GetHash()] = &PointPool::Take(existing.GetPoint());
		}
		return *m_ToPoints[existing.GetHash()];
	}

public:
	Shape & Duplicate(const Shape& toDup)
	{
		ShapeEdge::ResetNextHashCounter();
		ShapePoint::ResetNextHashCounter();

		auto& dup = ShapePool::Take();

		for (auto f : toDup.GetFaces())
		{
			auto& fDup = FacePool::Take();

			auto& points = f->GetPointObjects();
			auto& edges = f->GetEdgeObjects();

			for (auto i = 0u; i < points.size(); i++)
			{
				auto& p = *points[i];
				auto& e = *edges[i];

				if (!e.HashIsAssigned())
				{
					e.AssignHash();
					m_ToEdges[e.GetHash()] = &EdgePool::Take(GetNewPoint(p), GetNewPoint(*points[(i + 1u) % points.size()]));
				}

				fDup.AddPoint(*m_ToPoints[p.GetHash()], *m_ToEdges[e.GetHash()]);
			}

			fDup.SetNormal(f->GetNormal());
			dup.AddFace(fDup);
		}

		dup.CollectShapeElementsAndResetHashes();
		dup.ResetBeenCollectedFlag();

		ShapePoint::ResetHashes(toDup.GetPointObjects());
		ShapeEdge::ResetHashes(toDup.GetEdgeObjects());

		return dup;
	}

private:
	DynamicArray<ShapeEdge*> m_ToEdges;
	DynamicArray<ShapePoint*> m_ToPoints;
};
