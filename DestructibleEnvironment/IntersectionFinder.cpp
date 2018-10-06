#include "IntersectionFinder.h"
#include "Face.h"
#include "Transform.h"
#include "Shape.h"

bool IntersectionFinder::LineIsIntersectedWithFace(const Face& face, const Vector3& edgeP0, const Vector3& edgeP1, Vector3& intPoint)
{
	if (Vector3::LinePlaneIntersection(face.GetPlaneP0(), face.GetNormal(), edgeP0, edgeP1, intPoint))
		return face.PointIsInsideFace(intPoint);

	return false;
}

void IntersectionFinder::FindFaceEdgeIntersections(const std::vector<Vector3>& transformedPoints, const std::vector<int>& edgeIndexes, Face& face, std::vector<FaceEdgeIntersection<Vector3>>& inters)
{
	Vector3 intPoint;
	auto c = edgeIndexes.size();

	for (auto i = 0U; i < c; i++)
	{
		auto& p0 = transformedPoints[edgeIndexes[i]];
		auto& p1 = transformedPoints[edgeIndexes[(i + 1) % c]];

		if (LineIsIntersectedWithFace(face, p0, p1, intPoint))
			inters.emplace_back(FaceEdgeIntersection<Vector3>(face, i, m_ActiveTransform->ToWorldPosition(intPoint)));
	}
}

void IntersectionFinder::FindFaceEdgeIntersections(Face& face1, Face& face2, std::vector<FaceEdgeIntersection<Vector3>>& inters)
{
	// testing faces 1s edges against face 2
	m_ActiveTransform = &m_Shape2->GetTransform();
	FindFaceEdgeIntersections(m_Shape1sTransformedPoints, face1.GetSharedPoints(), face2, inters);

	// testing face 2s edges against face 1
	m_ActiveTransform = &m_Shape1->GetTransform();
	FindFaceEdgeIntersections(m_Shape2sTransformedPoints, face2.GetSharedPoints(), face1, inters);
}

void IntersectionFinder::FindFaceFaceIntersection(Face& face1, Face& face2, std::vector<FaceFaceIntersection<Vector3>>& inters)
{
	m_FaceEdgeIntersections.clear();
	FindFaceEdgeIntersections(face1, face2, m_FaceEdgeIntersections);

	if (m_FaceEdgeIntersections.size() == 2)
		inters.emplace_back(FaceFaceIntersection<Vector3>(face1, face2, m_FaceEdgeIntersections[0], m_FaceEdgeIntersections[1]));
}

void IntersectionFinder::FindFaceFaceIntersections(std::vector<FaceFaceIntersection<Vector3>>& inters)
{
	auto& faces1 = m_Shape1->GetFaces();
	auto& faces2 = m_Shape2->GetFaces();

	for (auto it1 = faces1.begin(); it1 != faces1.end(); it1++)
	{
		// TODO - this is where i can key into a partition to get the faces in shape 2 that are
		// nearby the face i in shape 1
		for (auto it2 = faces2.begin(); it2 != faces2.end(); it2++)
			FindFaceFaceIntersection(**it1, **it2, inters);
	}
}

void IntersectionFinder::FindFaceEdgeIntersections(std::vector<FaceEdgeIntersection<Vector3>>& inters)
{
	auto& faces1 = m_Shape1->GetFaces();
	auto& faces2 = m_Shape2->GetFaces();

	for (auto it1 = faces1.begin(); it1 != faces1.end(); it1++)
	{
		// TODO - this is where i can key into a partition to get the faces in shape 2 that are
		// nearby the face i in shape 1
		for (auto it2 = faces2.begin(); it2 != faces2.end(); it2++)
			FindFaceEdgeIntersections(**it1, **it2, inters);
	}
}

void IntersectionFinder::CommonInit(Shape& shape1, Shape& shape2)
{
	m_Shape1 = &shape1;
	m_Shape2 = &shape2;

	m_Shape1sTransformedPoints.clear();
	m_Shape2sTransformedPoints.clear();

	auto& t1 = m_Shape1->GetTransform();
	auto& t2 = m_Shape2->GetTransform();

	auto s1ToS2 = t2.GetWorldToLocalMatrix() * t1.GetLocalToWorldMatrix();
	auto s2ToS1 = t1.GetWorldToLocalMatrix() * t2.GetLocalToWorldMatrix();

	auto& shape1Points = m_Shape1->GetCachedPoints();
	auto& shape2Points = m_Shape2->GetCachedPoints();

	for (auto it = shape1Points.begin(); it != shape1Points.end(); it++)
		m_Shape1sTransformedPoints.emplace_back(s1ToS2 * (*it));

	for (auto it = shape2Points.begin(); it != shape2Points.end(); it++)
		m_Shape2sTransformedPoints.emplace_back(s2ToS1 * (*it));
}

void IntersectionFinder::FindFaceFaceIntersections(Shape& shape1, Shape& shape2, std::vector<FaceFaceIntersection<Vector3>>& inters)
{
	CommonInit(shape1, shape2);
	FindFaceFaceIntersections(inters);
}

void IntersectionFinder::FindFaceEdgeIntersections(Shape& shape1, Shape& shape2, std::vector<FaceEdgeIntersection<Vector3>>& inters)
{
	CommonInit(shape1, shape2);
	FindFaceEdgeIntersections(inters);
}