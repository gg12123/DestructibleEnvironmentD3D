#pragma once
#include "Vector3.h"

class Face;

class FaceFaceIntersection
{
public:
	Face * PiercedFace1;
	int PiercingEdge1;
	Vector3 Position1;

	Face * PiercedFace2;
	int PiercingEdge2;
	Vector3 Position2;
};
