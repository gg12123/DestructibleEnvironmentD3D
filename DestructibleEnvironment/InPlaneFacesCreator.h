#pragma once
#include "IntersectionLoop.h"
#include "Face.h"
#include "CollectionU.h"
#include "ShapeElementPool.h"
#include "NewShapeGeometryCreator.h"
#include "MapToNewEdges.h"

class InPlaneFaceCreator
{
private:
	int DirForFaceAbove(const Plane& splitPlane, const IntersectionLoop& loop)
	{
		auto n = splitPlane.GetNormal();
		auto& inters = loop.GetIntersections();

		auto crossSum = 0.0f;
		for (auto i = 0u; i < inters.size(); i++)
		{
			auto& p0 = inters[CollectionU::GetPrevIndex(inters, i)].GetIntPoint();
			auto& p1 = inters[i].GetIntPoint();
			auto& p2 = inters[CollectionU::GetNextIndex(inters, i)].GetIntPoint();

			auto d0 = (p1 - p0).Normalized();
			auto d1 = (p2 - p1).Normalized();

			crossSum += Vector3::Dot(n, Vector3::Cross(d0, d1));
		}

		return crossSum > 0.0f ? 1 : -1;
	}

	template<PointPlaneRelationship r>
	Face& CreateFace(int dir, const IntersectionLoop& loop, const Vector3& normal)
	{
		auto c = loop.GetCount();
		auto& inters = loop.GetIntersections();
		auto curr = 0;

		auto& newFace = FacePool::Take();

		for (int i = 0; i < c; i++)
		{
			curr = (curr + dir + c) % c;
			auto next = (curr + dir + c) % c;

			auto& p = m_NewPoints->GetPoint<r>(inters[curr].GetEdge());
			auto& pNext = m_NewPoints->GetPoint<r>(inters[next].GetEdge());

			newFace.AddPoint(p, m_NewEdges->GetNewEdge(p, pNext));
		}

		newFace.SetNormal(normal);
		return newFace;
	}

public:
	void Create(const IntersectionLoop& loop, const Plane& splitPlane, std::vector<Face*>& newFacesAbove, std::vector<Face*>& newFacesBelow)
	{
		auto aboveDir = DirForFaceAbove(splitPlane, loop);
		
		newFacesAbove.emplace_back(&CreateFace<PointPlaneRelationship::PointsAbove>(aboveDir, loop, -splitPlane.GetNormal()));
		newFacesBelow.emplace_back(&CreateFace<PointPlaneRelationship::PointsBelow>(-aboveDir, loop, splitPlane.GetNormal()));
	}

private:
	const MapToNewPoints* m_NewPoints;
	const MapToNewEdges* m_NewEdges;
};
