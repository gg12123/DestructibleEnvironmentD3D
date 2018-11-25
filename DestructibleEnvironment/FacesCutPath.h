#pragma once
#include <vector>
#include "CutPathElement.h"
#include "Vector3.h"
#include "MapToShapePointOnReversedFace.h"
#include "FaceRelationshipWithOtherShape.h"

class FacesCutPath
{
public:
	const CutPathElement& GetElement(int index) const
	{
		return (*m_CutPath)[index];
	}

	const CutPathElement& GetFirstElement() const
	{
		return GetElement(m_FirstIndex);
	}

	const CutPathElement& GetFinalElement() const
	{
		return GetElement(m_FinalIndex);
	}

	int GetFirstIndex() const
	{
		return m_FirstIndex;
	}

	int GetFinalIndex() const
	{
		return m_FinalIndex;
	}

	Vector3 GetDirToNext(int index, int travelDir) const
	{
		return travelDir == 1 ?
			GetElement(GetNextIndex(index, travelDir)).GetDirFromPrev() :
			-GetElement(index).GetDirFromPrev();
	}

	int GetNextIndex(int curr, int travelDir) const
	{
		auto c = m_CutPath->size();
		return (curr + travelDir + c) % c;
	}

	void SetBeenUsedToGenInsideFace()
	{
		m_BeenUsedToGenInside = true;
	}

	void SetBeenUsedToGetOutsideFace()
	{
		m_BeenUsedToGenOutside = true;
	}

	bool BeenUsedToGenInsideFace() const
	{
		return m_BeenUsedToGenInside;
	}

	bool BeenUsedToGenOutsideFace() const
	{
		return m_BeenUsedToGenOutside;
	}

	template<FaceRelationshipWithOtherShape inside>
	static inline ShapePoint& GetNewPointFromCpPoint(ShapePoint& pIn, const MapToShapePointOnReversedFace& map)
	{
		return pIn;
	}

	template<>
	static inline ShapePoint& GetNewPointFromCpPoint<FaceRelationshipWithOtherShape::NotInIntersection>(ShapePoint& pIn, const MapToShapePointOnReversedFace& map)
	{
		return map.GetPointOnReversedFace(pIn);
	}

	void Init(int first, int final, const std::vector<CutPathElement>& cp)
	{
		m_FirstIndex = first;
		m_FinalIndex = final;
		m_CutPath = &cp;

		m_BeenUsedToGenInside = false;
		m_BeenUsedToGenOutside = false;
	}

private:
	int m_FirstIndex;
	int m_FinalIndex;

	bool m_BeenUsedToGenInside;
	bool m_BeenUsedToGenOutside;

	const std::vector<CutPathElement>* m_CutPath;
};
