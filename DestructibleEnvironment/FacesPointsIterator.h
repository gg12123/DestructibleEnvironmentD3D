#pragma once
#include "FacesOriginalPointsIterator.h"
#include "FacesCutPathIterator.h"
#include "MapToFacesCutPath.h"
#include "FaceRelationshipWithOtherShape.h"

class FacesPointsIterator
{
private:
	float N0DotD0(const FacesCutPath& startCp) const
	{
		auto& firstElement = startCp.GetFirstElement();

		auto& d0 = firstElement.GetPiercingEdge().GetDirection(*m_OriginalFace);
		auto& n0 = firstElement.GetPiercedFace().GetNormal();

		return Vector3::Dot(n0, d0);
	}

	ShapePoint & FindStartPointInside(const FacesCutPath& startCp) const
	{
		return N0DotD0(startCp) > 0.0f ? startCp.GetFirstElement().GetPoint() : startCp.GetFinalElement().GetPoint();
	}

	ShapePoint & FindStartPointOutside(const FacesCutPath& startCp) const
	{
		return N0DotD0(startCp) > 0.0f ? startCp.GetFinalElement().GetPoint() : startCp.GetFirstElement().GetPoint();
	}

	int IndexOfP(const ShapePoint& p, const ShapeEdge& edgeContainingP)
	{
		auto& start = edgeContainingP.GetStart(*m_OriginalFace);
		auto& end = edgeContainingP.GetEnd(*m_OriginalFace);

		auto startPointIndex = edgeContainingP.GetIndex(*m_OriginalFace);
		if (&p == &start)
		{
			return startPointIndex;
		}
		else if (&p == &end)
		{
			return m_OriginalFace->NextPointIndex(startPointIndex);
		}
		assert(false);
		return -1;
	}

	template<FaceRelationshipWithOtherShape inside>
	static inline void SetBeenUsedToGen(FacesCutPath& cp)
	{
		cp.SetBeenUsedToGenInsideFace();
	}

	template<>
	static inline void SetBeenUsedToGen<FaceRelationshipWithOtherShape::NotInIntersection>(FacesCutPath& cp)
	{
		cp.SetBeenUsedToGetOutsideFace();
	}

	template<FaceRelationshipWithOtherShape inOrOut>
	void Iterate(ShapePoint& sp)
	{
		auto iteratingCp = true;

		ShapePoint* p = &sp;
		ShapeEdge* piercingEdge = nullptr;

		do
		{
			if (iteratingCp)
			{
				auto& cp = m_MapToCutPaths->GetPath(*m_OriginalFace, *p);
				SetBeenUsedToGen<inOrOut>(cp);

				m_CPIterator.IteratePath<inOrOut>(cp, *p);

				piercingEdge = &m_CPIterator.GetEndEdge();
				p = &m_CPIterator.GetNextPoint();

				auto& se = piercingEdge->GetSplitEdge();
				iteratingCp = (p != &se.GetP0() && p != &se.GetP1());
			}
			else
			{
				p = &m_OPIterator.Iterate<inOrOut>(IndexOfP(*p, *piercingEdge));
				iteratingCp = true;
			}
		} while (p != &sp);
	}

public:
	void IterateInside(const FacesCutPath& startCp)
	{
		auto& sp = FindStartPointInside(startCp);
		Iterate<FaceRelationshipWithOtherShape::InIntersection>(sp);
	}

	void IterateOutside(const FacesCutPath& startCp)
	{
		auto& sp = FindStartPointOutside(startCp);
		Iterate<FaceRelationshipWithOtherShape::NotInIntersection>(sp);
	}

	void InitMaps(const MapToShapePointOnReversedFace& map, MapToNewEdges& edgeMap, const MapToFacesCutPath& mapToFcp)
	{
		m_MapToCutPaths = &mapToFcp;
		m_OPIterator.InitMaps(map, edgeMap);
		m_CPIterator.InitMaps(map, edgeMap);
	}

	void InitFaces(const Face& orig, Face& newFace)
	{
		m_OriginalFace = &orig;
		m_OPIterator.InitFaces(orig, newFace);
		m_CPIterator.InitFaces(orig, newFace);
	}

private:
	FacesOriginalPointsIterator m_OPIterator;
	FacesCutPathIterator m_CPIterator;

	const Face* m_OriginalFace = nullptr;
	const MapToFacesCutPath* m_MapToCutPaths = nullptr;
};
