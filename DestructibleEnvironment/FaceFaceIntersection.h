#pragma once
#include "Vector3.h"

class Face;

class FaceEdgeIntersection
{
public:
	Face * PiercedFace;
	int PiercingEdge;
	Vector2 Position;
};

class FaceFaceIntersection
{
public:
	Face * Face1;
	Face * Face2;

	FaceEdgeIntersection Intersection1;
	FaceEdgeIntersection Intersection2;
};
