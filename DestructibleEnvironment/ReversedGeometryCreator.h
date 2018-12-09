#pragma once
#include "ShapeEdgesCreator.h"
#include "ShapePoint.h"
#include "CutPathElement.h"
#include "MapToShapePointOnReversedFace.h"

class ReversedGeometryCreator
{
private:
	ShapePoint& CreatePointCommon(const ShapePoint& from)
	{
		auto p = new ShapePoint(from.GetPoint()); // TODO - get form pool
		m_MapToReversed.Add(from, *p);
		p->AssignHash();
		return *p;
	}

	ShapePoint & TryCreatePointOnReversed(ShapePoint& from)
	{
		if (!from.HashIsAssigned())
		{
			from.AssignHash();
			CreatePointCommon(from);
		}
		return m_MapToReversed.GetPointOnReversedFace(from);
	}

	ShapePoint & CreatePointOnReversed(const ShapePoint& from)
	{
		assert(from.HashIsAssigned());
		return CreatePointCommon(from);;
	}

public:
	void CreateReversedCutPath(const std::vector<CutPathElement>& cp)
	{
		// The cp points have already been assigned with hashes so the
		// try create point method wont work here. But it is ok to assume
		// that none of the cp points have been mapped to a reversed equivalent.

		auto prev = &CreatePointOnReversed(cp[0].GetPoint());
		auto first = prev;

		ShapePoint* curr = nullptr;
		for (auto i = 1U; i < cp.size(); i++)
		{
			curr = &CreatePointOnReversed(cp[i].GetPoint());
			m_EdgeCreator->CreateEdge(*prev, *curr);
			prev = curr;
		}

		m_EdgeCreator->CreateEdge(*curr, *first);
	}

	void CreateReversedFace(const Face& face)
	{
		auto& points = face.GetPointObjects();

		for (auto i = 0U; i < points.size(); i++)
		{
			auto next = CollectionU::GetNextIndex(points, i);

			auto& p0 = TryCreatePointOnReversed(*points[i]);
			auto& p1 = TryCreatePointOnReversed(*points[next]);

			if (!m_EdgeCreator->EdgeExistsBetween(p0, p1))
				m_EdgeCreator->CreateEdge(p0, p1);
		}

		// TODO - pool
		auto reversedFace = new Face();

		for (int i = points.size() - 1; i >= 0; i--)
		{
			int prev = CollectionU::GetPrevIndex(points, i);

			auto& pRev0 = m_MapToReversed.GetPointOnReversedFace(*points[i]);
			auto& pRev1 = m_MapToReversed.GetPointOnReversedFace(*points[prev]);

			auto& e = m_EdgeCreator->GetMapToNewEdges().GetNewEdge(pRev0, pRev1);

			reversedFace->AddPoint(pRev0, e);
		}

		reversedFace->SetNormal(-face.GetNormal());
	}

	void Init(ShapeEdgesCreator& edgeCreator)
	{
		m_EdgeCreator = &edgeCreator;
	}

	const auto& GetMapToReversed() const
	{
		return m_MapToReversed;
	}

private:
	ShapeEdgesCreator * m_EdgeCreator;
	MapToShapePointOnReversedFace m_MapToReversed;
};