#include "CollisionDetector.h"
#include "Shape.h"
#include "MathUtils.h"

void CollisionDetector::FindPointFaceCollision(const std::vector<Vector3>& faceNormals, const std::vector<Vector3> faceP0s, const Vector3& point)
{
	auto indexOfClosest = 0;
	auto currIndex = 0;
	auto closestDist = MathUtils::Infinity;

	auto itN = faceNormals.begin();
	auto itP = faceP0s.begin();

	for (; itN != faceNormals.end(); itN++, itP++)
	{
		auto comp = Vector3::Dot(point - *itP, *itN);

		if (comp > 0.0f)
			return;

		auto absComp = fabs(comp);

		if (absComp < closestDist)
		{
			closestDist = absComp;
			indexOfClosest = currIndex;
		}

		currIndex++;
	}

	auto& coll = m_DataPool.GetNextData();

	coll.Normal = faceNormals[indexOfClosest];
	coll.Position = point;
	coll.Penetration = closestDist;

	m_FoundCollisions.emplace_back(&coll);
}

// points must be in the shapes space
void CollisionDetector::FindPointFaceCollisions(Shape& shapeFaces, ArrayWrapper<Vector3, ShapeConstants::MaxNumPoints>& points)
{
	auto& faceNormals = shapeFaces.GetCachedFaceNormals();
	auto& faceP0s = shapeFaces.GetCachedFaceP0s();

	auto data = points.GetData();
	auto count = points.GetCurrCount();

	for (auto i = 0; i < count; i++)
		FindPointFaceCollision(faceNormals, faceP0s, data[i]);
}

CollisionData * CollisionDetector::FindCollision(Shape& shape1, Shape& shape2)
{
	m_FoundCollisions.clear();
	m_DataPool.Reset();

	m_Shape1TransformedPoints.Clear();
	m_Shape2TransformedPoints.Clear();

	auto& t1 = shape1.GetTransform();
	auto& t2 = shape2.GetTransform();

	auto shape1To2 = t2.GetWorldToLocalMatrix() * t1.GetLocalToWorldMatrix();
	auto shape2To1 = t1.GetWorldToLocalMatrix() * t2.GetLocalToWorldMatrix();

	auto& shape1Points = shape1.GetCachedPoints();
	auto& shape2Points = shape2.GetCachedPoints();

	for (auto it = shape1Points.begin(); it != shape1Points.end(); it++)
		m_Shape1TransformedPoints.Add(shape1To2 * (*it));

	for (auto it = shape2Points.begin(); it != shape2Points.end(); it++)
		m_Shape2TransformedPoints.Add(shape2To1 * (*it));

	FindPointFaceCollisions(shape1, m_Shape2TransformedPoints);
	FindPointFaceCollisions(shape2, m_Shape1TransformedPoints);


}