#include "IntersectionFinder.h"
#include "Face.h"
#include "Shape.h"

Vector3 IntersectionFinder::TransformIntPoint(const Vector3& p)
{
	return m_ActiveTransform->ToWorldPosition(p);
}

bool IntersectionFinder::EdgeIsIntersectedWithFace(const Face& face, const Vector3& edgeP0, const Vector3& edgeP1, Vector3& intPoint)
{
	auto n = face.GetNormal();
	auto p0 = face.GetPlaneP0();

	if (Vector3::Dot(edgeP0 - p0, n) * Vector3::Dot(edgeP1 - p0, n) <= 0.0f)
	{
		if (Vector3::LinePlaneIntersection(face.GetPlaneP0(), face.GetNormal(), edgeP0, edgeP1, intPoint))
			return face.PointIsInsideFace(intPoint);
	}

	return false;
}

void IntersectionFinder::FindFaceEdgeIntersections(const std::vector<Vector3>& shapeEdgesTransformedPoints, const Shape& shapeEdges, const Shape& shapeFaces, std::vector<EdgeFaceIntersection>& inters)
{
	auto& edgeObjects = shapeEdges.GetEdgeObjects();
	auto& edgeIndexs = shapeEdges.GetEdgeIndexes();

	Vector3 intPoint;

	for (auto i = 0U; i < edgeObjects.size(); i++)
	{
		auto j = 2U * i;

		auto& p0 = shapeEdgesTransformedPoints[edgeIndexs[j]];
		auto& p1 = shapeEdgesTransformedPoints[edgeIndexs[j + 1U]];

		// now would be the time to query a parition for nearby faces

		auto& faces = shapeFaces.GetFaces();

		for (auto it = faces.begin(); it != faces.end(); it++)
		{
			if (EdgeIsIntersectedWithFace(**it, p0, p1, intPoint))
				inters.emplace_back(EdgeFaceIntersection(**it, *edgeObjects[i], TransformIntPoint(intPoint)));
		}
	}
}

void IntersectionFinder::CommonInit(Shape& shape1, Shape& shape2)
{
	m_Shape1sTransformedPoints.clear();
	m_Shape2sTransformedPoints.clear();

	auto& t1 = shape1.GetTransform();
	auto& t2 = shape2.GetTransform();

	auto s1ToS2 = t2.GetWorldToLocalMatrix() * t1.GetLocalToWorldMatrix();
	auto s2ToS1 = t1.GetWorldToLocalMatrix() * t2.GetLocalToWorldMatrix();

	auto& shape1Points = shape1.GetCachedPoints();
	auto& shape2Points = shape2.GetCachedPoints();

	for (auto it = shape1Points.begin(); it != shape1Points.end(); it++)
		m_Shape1sTransformedPoints.emplace_back(s1ToS2 * (*it));

	for (auto it = shape2Points.begin(); it != shape2Points.end(); it++)
		m_Shape2sTransformedPoints.emplace_back(s2ToS1 * (*it));
}

void IntersectionFinder::FindEdgeFaceIntersections(Shape& shape1, Shape& shape2, std::vector<EdgeFaceIntersection>& inters)
{
	CommonInit(shape1, shape2);

	m_ActiveTransform = &shape2.GetTransform();
	FindFaceEdgeIntersections(m_Shape1sTransformedPoints, shape1, shape2, inters);

	m_ActiveTransform = &shape1.GetTransform();
	FindFaceEdgeIntersections(m_Shape2sTransformedPoints, shape2, shape1, inters);
}

void IntersectionFinder::FindEdgeFaceIntersectionsLocalToFaces(Shape& shape1, Shape& shape2, std::vector<EdgeFaceIntersection>& inters)
{
	CommonInit(shape1, shape2);
	m_ActiveTransform = &m_IdentityTransform;

	FindFaceEdgeIntersections(m_Shape1sTransformedPoints, shape1, shape2, inters);
	FindFaceEdgeIntersections(m_Shape2sTransformedPoints, shape2, shape1, inters);
}