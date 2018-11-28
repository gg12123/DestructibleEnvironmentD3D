#pragma once
#include <memory>
#include "ShapeEdge.h"
#include "SplitShapeEdge.h"
#include "MapToNewEdges.h"
#include "MapToShapePointOnReversedFace.h"
#include "CutPathElement.h"
#include "PoolOfRecyclables.h"

class ShapeEdgesCreator
{
private:
	void CreateEdgesAlongCutPathImp(const std::vector<CutPathElement>& cp)
	{
		m_SplitEdgesPool->Reset();

		for (auto i = 0U; i < cp.size(); i++)
		{
			auto nextI = (i + 1U) % cp.size();

			auto& p0 = cp[i].GetPoint();
			auto& p1 = cp[nextI].GetPoint();
			auto& dir = cp[nextI].GetDirFromPrev();

			CreateEdge(p0, p1, dir);

			auto& pe = cp[i].GetPiercingEdge();

			if (!pe.IsSplit())
			{
				auto& se = m_SplitEdgesPool->Recycle();
				se.Init(pe);
				pe.SetSplitEdge(se);
			}

			pe.GetSplitEdge().AddElement(cp[i]);
		}
	}

	ShapePoint& GetShapePoint(const SplitShapeEdge& edge, ShapePoint& p, bool inside)
	{
		if (inside)
			return p;

		if (&p == &edge.GetP0() || &p == &edge.GetP1())
			return p;

		return m_MapToRev->GetPointOnReversedFace(p);
	}

	void CreateEdgesAlongSplitEdge(bool inside, const SplitShapeEdge& edge)
	{
		auto create = inside ? edge.FirstEdgeFromP0IsInside() : edge.FirstEdgeFromP0IsOutside();
		auto& sePoints = edge.GetPointsSortedFromP0();
		auto dir = edge.GetEdge().GetDirFromP0ToP1();

		for (auto i = 0U; i < sePoints.size() - 1U; i++)
		{
			if (create)
			{
				auto& p0 = GetShapePoint(edge, *sePoints[i], inside);
				auto& p1 = GetShapePoint(edge, *sePoints[i + 1U], inside);

				CreateEdge(p0, p1, dir);
			}

			create = !create;
		}
	}

	void InitSplitEdges()
	{
		for (auto it = m_SplitEdgesPool->Begin(); it != m_SplitEdgesPool->End(); it++)
			it->OnAllElementsAdded();
	}

	void CreateEdgesAlongSplitEdges(bool inside)
	{
		for (auto it = m_SplitEdgesPool->Begin(); it != m_SplitEdgesPool->End(); it++)
			CreateEdgesAlongSplitEdge(inside, *it);
	}

public:
	void CreateEdgesAlongCutPath(const std::vector<CutPathElement>& cp)
	{
		CreateEdgesAlongCutPathImp(cp);
		InitSplitEdges();
	}

	void CreateInsideEdges()
	{
		CreateEdgesAlongSplitEdges(true);
	}

	void CreateOutsideEdges(const MapToShapePointOnReversedFace& mapToRev)
	{
		m_MapToRev = &mapToRev;
		CreateEdgesAlongSplitEdges(false);
	}

	// called by the reversing code
	void CreateEdge(ShapePoint& p0, ShapePoint& p1, const Vector3& dirToP1)
	{
		auto& edge = *(new ShapeEdge(p0, p1, dirToP1)); // TODO - pool
		m_MapToEdges.AddNewEdge(p0, p1, edge);
	}

	const auto& GetMapToNewEdges() const
	{
		return m_MapToEdges;
	}

	bool EdgeExistsBetween(const ShapePoint& p0, const ShapePoint& p1) const
	{
		return m_MapToEdges.EdgeExistsBetween(p0, p1);
	}

	void ClearMap()
	{
		m_MapToEdges.Clear();
	}

private:
	MapToNewEdges m_MapToEdges;
	std::unique_ptr<PoolOfRecyclables<SplitShapeEdge>> m_SplitEdgesPool;
	const MapToShapePointOnReversedFace* m_MapToRev = nullptr;
};