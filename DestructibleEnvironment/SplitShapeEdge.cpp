#include "pch.h"
#include "SplitShapeEdge.h"
#include "ShapeEdge.h"
#include "ShapePoint.h"

ShapePoint & SplitShapeEdge::GetP0() const
{
	return m_Edge->GetP0();
}

ShapePoint & SplitShapeEdge::GetP1() const
{
	return m_Edge->GetP1();
}

static ShapePoint& RemoveClosest(std::vector<CutPathElement>& elements, const Vector3& p)
{
	auto closestDist = MathU::Infinity;
	std::vector<CutPathElement>::iterator itOfClosest = elements.begin();

	for (auto it = elements.begin(); it != elements.end(); it++)
	{
		auto dist = (p - it->GetPoint().GetPoint()).MagnitudeSqr();

		if (dist < closestDist)
		{
			closestDist = dist;
			itOfClosest = it;
		}
	}

	auto& closestPoint = itOfClosest->GetPoint();
	elements.erase(itOfClosest);
	return closestPoint;
}

void SplitShapeEdge::OnAllElementsAdded()
{
	auto& p0 = GetP0();
	auto p0point = p0.GetPoint();

	// n^2 sort but there will usually only be 1 or 2 elements to sort.

	m_PointsSortedFromP0.emplace_back(&p0);

	while (m_Elements.size() > 0U)
		m_PointsSortedFromP0.emplace_back(&RemoveClosest(m_Elements, p0point));

	m_PointsSortedFromP0.emplace_back(&GetP1());
}