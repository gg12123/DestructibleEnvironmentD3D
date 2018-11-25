#pragma once
#include <array>
#include "Constants.h"
#include "FaceRelationshipWithOtherShape.h"
#include "Face.h"

class MapToFaceRelationship
{
public:
	FaceRelationshipWithOtherShape GetRelationship(const Face& f) const
	{
		return m_Map[f.GetHash()];
	}

	void SetRelationship(const Face& f, FaceRelationshipWithOtherShape r)
	{
		m_Map[f.GetHash()] = r;
	}

private:
	std::array<FaceRelationshipWithOtherShape, Constants::MaxNumPoints> m_Map;
};
