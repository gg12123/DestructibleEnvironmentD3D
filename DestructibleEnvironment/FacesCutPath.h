#pragma once
#include <vector>
#include "CutPathElement.h"
#include "Vector3.h"
#include "MapToShapePointOnReversedFace.h"

class FacesCutPath
{
public:
	const CutPathElement& GetElement(int index) const
	{
		auto& cp = *m_CutPath;
		return cp[index];
	}

	const CutPathElement& GetFirstElement() const
	{
	}

	const CutPathElement& GetFinalElement() const
	{
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

	bool SetBeenUsedToGenInsideFace()
	{

	}

	bool SetBeenUsedToGetOutsideFace()
	{

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

	}

private:

	int m_FirstIndex;
	int m_FinalIndex;

	const std::vector<CutPathElement>* m_CutPath;
};
