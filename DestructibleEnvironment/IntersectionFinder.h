#pragma once
#include <vector>
#include "Vector3.h"
#include "FaceFaceIntersection.h"

class Face;
class Shape;
class Transform;

class IntersectionFinder
{
public:
	// for splitting
	void FindFaceFaceIntersections(Shape& shape1, Shape& shape2, std::vector<FaceFaceIntersection<Vector3>>& inters);

	// for collision detection
	void FindFaceEdgeIntersections(Shape& shape1, Shape& shape2, std::vector<FaceEdgeIntersection<Vector3>>& inters);

private:
	void CommonInit(Shape& shape1, Shape& shape2);

	bool EdgeIsIntersectedWithFace(const Face& face, const Vector3& edgeP0, const Vector3& edgeP1, Vector3& intPoint);

	void FindFaceEdgeIntersections(const std::vector<Vector3>& transformedPoints, const std::vector<int>& edgeIndexes, const Face& face, std::vector<FaceEdgeIntersection<Vector3>>& inters);
	void FindFaceEdgeIntersections(const Face& face1, const Face& face2, std::vector<FaceEdgeIntersection<Vector3>>& inters);
	void FindFaceFaceIntersection(const Face& face1, const Face& face2, std::vector<FaceFaceIntersection<Vector3>>& inters);
	void FindFaceFaceIntersections(std::vector<FaceFaceIntersection<Vector3>>& inters);
	void FindFaceEdgeIntersections(std::vector<FaceEdgeIntersection<Vector3>>& inters);

	std::vector<Vector3> m_Shape1sTransformedPoints;
	std::vector<Vector3> m_Shape2sTransformedPoints;

	Transform* m_ActiveTransform;

	Shape* m_Shape1;
	Shape* m_Shape2;

	std::vector<FaceEdgeIntersection<Vector3>> m_FaceEdgeIntersections;
};
