#include "IntersectionFinder.h"
#include "Face.h"

bool IntersectionFinder::EdgeIsIntersectedWithFace(Face& face, const Vector3& edgeP0, const Vector3& edgeP1, Vector3& intPoint)
{
	if (Vector3::LinePlaneIntersection(face.GetPlaneP0(), face.GetNormal(), edgeP0, edgeP1, intPoint))
		return face.PointIsInsideFace(intPoint);

	return false;
}