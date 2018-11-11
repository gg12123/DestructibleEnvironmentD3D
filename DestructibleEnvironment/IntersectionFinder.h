#pragma once
#include <vector>
#include "Vector3.h"
#include "FaceFaceIntersection.h"
#include "EdgeFaceIntersection.h"

class Face;
class Shape;
class Transform;

class IntersectionFinder
{
public:
	void FindEdgeFaceIntersections(Shape& shape1, Shape& shape2, std::vector<EdgeFaceIntersection>& inters);

	// for splitting
	void FindFaceFaceIntersections(Shape& shape1, Shape& shape2, std::vector<FaceFaceIntersection<Vector3>>& inters);

	// for collision detection
	void FindFaceEdgeIntersections(Shape& shape1, Shape& shape2, std::vector<FaceEdgeIntersection<Vector3>>& inters);

	static bool LineIsIntersectedWithFace(const Face& face, const Vector3& lineP0, const Vector3& lineP1, Vector3& intPoint);

	const auto& GetShape1sTransformedPoints() const
	{
		return m_Shape1sTransformedPoints;
	}

	const auto& GetShape2sTransformedPoints() const
	{
		return m_Shape2sTransformedPoints;
	}

private:
	void CommonInit(Shape& shape1, Shape& shape2);

	void FindFaceEdgeIntersections(const std::vector<Vector3>& transformedPoints, Face& faceEdges, Face& face, std::vector<FaceEdgeIntersection<Vector3>>& inters);
	void FindFaceEdgeIntersections(Face& face1, Face& face2, std::vector<FaceEdgeIntersection<Vector3>>& inters);
	void FindFaceFaceIntersection(Face& face1, Face& face2, std::vector<FaceFaceIntersection<Vector3>>& inters);
	void FindFaceFaceIntersections(std::vector<FaceFaceIntersection<Vector3>>& inters);
	void FindFaceEdgeIntersections(std::vector<FaceEdgeIntersection<Vector3>>& inters);

	std::vector<Vector3> m_Shape1sTransformedPoints;
	std::vector<Vector3> m_Shape2sTransformedPoints;

	Transform* m_ActiveTransform;

	Shape* m_Shape1;
	Shape* m_Shape2;

	std::vector<FaceEdgeIntersection<Vector3>> m_FaceEdgeIntersections;
};
