#pragma once
#include "DynamicArray.h"
#include "Constants.h"
#include "FaceRelationshipWithOtherShape.h"
#include "Face.h"

class MapToFaceRelationship
{
public:
	FaceRelationshipWithOtherShape GetRelationship(const Face& f)
	{
		return m_Map[f.GetHash()];
	}

	void SetRelationship(const Face& f, FaceRelationshipWithOtherShape r)
	{
		m_Map[f.GetHash()] = r;
	}

private:
	DynamicArray<FaceRelationshipWithOtherShape> m_Map;
};
