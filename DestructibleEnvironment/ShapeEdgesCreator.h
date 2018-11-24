#pragma once
#include <memory>
#include "ShapeEdge.h"
#include "SplitShapeEdge.h"
#include "MapToNewEdges.h"
#include "CutPathElement.h"
#include "PoolOfRecyclables.h"

class ShapeEdgesCreator
{
private:
	void CreateEdgesAlongCutPath(const std::vector<CutPathElement>& cp)
	{
		m_SplitEdgesPool->Recycle();

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

	void CreateEdgesAlongSplitEdges()
	{
		for (auto it = m_SplitEdgesPool->Begin(); it != m_SplitEdgesPool->End(); it++)
		{
			it->OnAllElementsAdded();
			auto& sePoints = it->GetPointsSortedFromP0();
			auto dir = it->GetEdge().GetDirFromP0ToP1();

			for (auto i = 0U; i < sePoints.size() - 1U; i++)
				CreateEdge(*sePoints[i], *sePoints[i + 1U], dir);
		}
	}

public:
	void CreateEdges(const std::vector<CutPathElement>& cp)
	{
		CreateEdgesAlongCutPath(cp);
		CreateEdgesAlongSplitEdges();
	}

	// called by the reversing code
	void CreateEdge(const ShapePoint& p0, const ShapePoint& p1, const Vector3& dirToP1)
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
};