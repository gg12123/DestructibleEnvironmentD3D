#pragma once
#include "Vector3.h"

class Face;

template<class Tvec>
class FaceEdgeIntersection
{
public:
	Face * PiercedFace;
	Face * OtherFace;

	int PiercingEdge;
	Tvec Position;

	FaceEdgeIntersection(Face& f, Face& o, int pe, Tvec pos)
	{
		PiercedFace = &f;
		OtherFace = &o;
		PiercingEdge = pe;
		Position = pos;
	}
};

template<class Tvec>
class FaceFaceIntersection
{
public:
	Face * Face1;
	Face * Face2;

	FaceEdgeIntersection<Tvec> Intersection1;
	FaceEdgeIntersection<Tvec> Intersection2;

	FaceFaceIntersection(Face& face1, Face& face2, const FaceEdgeIntersection<Tvec>& int1, const FaceEdgeIntersection<Tvec>& int2)
	{
		Face1 = &face1;
		Face2 = &face2;

		Intersection1 = int1;
		Intersection2 = int2;
	}
};
