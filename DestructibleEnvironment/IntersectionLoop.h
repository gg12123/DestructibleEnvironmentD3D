#pragma once
#include "EdgeFaceIntersection.h"
#include "ShapeEdge.h"
#include "Face.h"
#include "CollectionU.h"

class IntersectionLoop
{
private:
	Face & GetFaceExitedOrEntered(int currIndex, int nextIndexAlong) const
	{
		auto& pe = GetPiercingEdge(currIndex);

		auto& f1 = pe.GetFace1();
		auto& f2 = pe.GetFace2();

		return GetPiercingEdge(nextIndexAlong).IsAttachedTo(f1) || &GetPiercedFace(nextIndexAlong) == &f1 ? f1 : f2;
	}

public:
	void AddIntersection(const EdgeFaceIntersection& inter)
	{
		m_Intersections.emplace_back(inter);
	}

	void Clear()
	{
		m_Intersections.clear();
	}

	Face& GetPiercedFace(int index) const
	{
		return m_Intersections[index].GetFace();
	}

	ShapeEdge& GetPiercingEdge(int index) const
	{
		return m_Intersections[index].GetEdge();
	}

	Face& GetFaceEntered(int index) const
	{
		auto next = CollectionU::GetNextIndex(m_Intersections, index);
		return GetFaceExitedOrEntered(index, next);
	}

	Face& GetFaceExited(int index) const
	{
		auto prev = CollectionU::GetNextIndex(m_Intersections, index);
		return GetFaceExitedOrEntered(index, prev);
	}

	int GetCount() const
	{
		return m_Intersections.size();
	}

	Vector3 GetIntPoint(int index) const
	{
		return m_Intersections[index].GetIntPoint();
	}

private:
	std::vector<EdgeFaceIntersection> m_Intersections;
};
