#pragma once
#include "Vector3.h"

class Face;

class IntersectionFinder
{
private:

	bool EdgeIsIntersectedWithFace(Face& face, const Vector3& edgeP0, const Vector3& edgeP1, Vector3& intPoint);
	void FindAndRegisterFaceFaceIntersection(Face& face1, Face& face2);
};
