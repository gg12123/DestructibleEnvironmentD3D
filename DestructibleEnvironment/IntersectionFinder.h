#pragma once
#include <vector>
#include "Vector3.h"
#include "EdgeFaceIntersection.h"
#include "Transform.h"

class Face;
class Shape;

class IntersectionFinder
{
public:
	IntersectionFinder()
	{
		m_IdentityTransform.SetPosition(Vector3::Zero());
		m_IdentityTransform.SetRotation(Quaternion::Identity());
	}

	// TODO - the shapes should be const
	void FindEdgeFaceIntersections(Shape& shape1, Shape& shape2, std::vector<EdgeFaceIntersection>& inters);
	void FindEdgeFaceIntersectionsLocalToFaces(Shape& shape1, Shape& shape2, std::vector<EdgeFaceIntersection>& inters);

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
	Vector3 TransformIntPoint(const Vector3& p);
	void FindFaceEdgeIntersections(const std::vector<Vector3>& shapeEdgesTransformedPoints, const Shape& shapeEdges, const Shape& shapeFaces, std::vector<EdgeFaceIntersection>& inters);
	static bool EdgeIsIntersectedWithFace(const Face& face, const Vector3& edgeP0, const Vector3& edgeP1, Vector3& intPoint);

	std::vector<Vector3> m_Shape1sTransformedPoints;
	std::vector<Vector3> m_Shape2sTransformedPoints;

	Transform* m_ActiveTransform;
	Transform m_IdentityTransform;

	std::vector<FaceEdgeIntersection<Vector3>> m_FaceEdgeIntersections;
};
