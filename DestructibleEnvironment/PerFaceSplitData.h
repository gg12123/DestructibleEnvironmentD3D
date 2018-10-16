#pragma once
#include "FaceRelationshipWithOtherShape.h"


class PerFaceSplitData
{
public:
	FaceRelationshipWithOtherShape RelationshipWithOtherShape;
	bool HasBeenAddedToDetachedFromList;
	bool Visited;

	PerFaceSplitData(FaceRelationshipWithOtherShape relationship)
	{
		RelationshipWithOtherShape = relationship;
		HasBeenAddedToDetachedFromList = false;
		Visited = false;
	}
};