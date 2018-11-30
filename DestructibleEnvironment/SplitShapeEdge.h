#pragma once
#include <vector>
#include <assert.h>
#include "CutPathElement.h"

class ShapePoint;
class ShapeEdge;

class SplitShapeEdge
{
public:
	void Init(const ShapeEdge& edge)
	{
		m_PointsSortedFromP0.clear();
		m_Elements.clear();
		m_Edge = &edge;
	}

	void AddElement(const CutPathElement& cpe)
	{
		m_Elements.emplace_back(cpe);
	}

	ShapePoint & GetNext(const CutPathElement& middleCurr, const ShapePoint& towards) const
	{
		assert(&towards == &GetP0() || &towards == &GetP1());

		auto i = &towards == &GetP0() ? middleCurr.GetIndexInSplitEdge() + 1 : middleCurr.GetIndexInSplitEdge() - 1;
		return *m_PointsSortedFromP0[i];
	}

	ShapePoint & GetNext(const ShapePoint& endCurr) const
	{
		assert(&endCurr == &GetP0() || &endCurr == &GetP1());
		return &endCurr == &GetP0() ? *m_PointsSortedFromP0[1] : *m_PointsSortedFromP0[m_PointsSortedFromP0.size() - 1U];
	}

	const auto& GetPointsSortedFromP0() const
	{
		return m_PointsSortedFromP0;
	}

	const auto& GetEdge() const
	{
		return *m_Edge;
	}

	bool FirstEdgeFromP0IsInside() const
	{

	}

	bool FirstEdgeFromP0IsOutside() const
	{
		return !FirstEdgeFromP0IsInside();
	}

	ShapePoint & GetP0() const;
	ShapePoint & GetP1() const;

	void OnAllElementsAdded();

private:
	std::vector<ShapePoint*> m_PointsSortedFromP0;
	std::vector<CutPathElement> m_Elements;
	const ShapeEdge* m_Edge = nullptr;
	bool m_FirstEdgeFromP0IsInside;
};
