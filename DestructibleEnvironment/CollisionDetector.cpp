#include "pch.h"
#include "CollisionDetector.h"
#include "Shape.h"
#include "MathUtils.h"
#include "Transform.h"

Vector3 CollisionDetector::TransformPoint(const Vector3& point)
{
	return m_ActiveTransform->ToWorldPosition(point);
}

Vector3 CollisionDetector::TransformDirection(const Vector3& dir)
{
	return m_ActiveTransform->ToWorldDirection(dir);
}

Vector3 CollisionDetector::InverseTransformPoint(const Vector3& point)
{
	return m_ActiveTransform->ToLocalPosition(point);
}

Vector3 CollisionDetector::InverseTransformDirection(const Vector3& dir)
{
	return m_ActiveTransform->ToLocalDirection(dir);
}

void CollisionDetector::FindPointFaceCollision(const std::vector<Vector3>& faceNormals, const std::vector<Vector3>& faceP0s, const Vector3& point)
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

	auto& coll = *m_DataPool->Recycle();

	coll.Normal1To2 = faceNormals[indexOfClosest];
	coll.Position = point;
	coll.Penetration = closestDist;

	m_FoundCollisions.emplace_back(&coll);
}

// points must be in the shapes space
void CollisionDetector::FindPointFaceCollisions(Shape& shapeFaces, ArrayWrapper<Vector3, Constants::MaxNumPoints>& points)
{
	auto& faceNormals = shapeFaces.GetCachedFaceNormals();
	auto& faceP0s = shapeFaces.GetCachedFaceP0s();

	auto data = points.GetData();
	auto count = points.GetCurrCount();

	for (auto i = 0; i < count; i++)
		FindPointFaceCollision(faceNormals, faceP0s, data[i]);
}

void CollisionDetector::FindEdgeCollision(const std::vector<Vector3>& facePoints, const Vector3& faceNormal, const Vector3& edgeP0, const Vector3& edgeP1)
{
	auto closestEdgeIndex = 0;
	auto closestDist = MathUtils::Infinity;

	auto intPoint = Vector3::LinePlaneIntersection(facePoints[0], faceNormal, edgeP0, edgeP1);

	auto size = facePoints.size();

	for (auto i = 0U; i < size; i++)
	{
		auto next = (i + 1U) % size;

		auto& P0 = facePoints[i];
		auto& P1 = facePoints[next];

		auto n = Vector3::Normalize(Vector3::Cross(faceNormal, P1 - P0));

		auto comp = Vector3::Dot(n, intPoint - P0);

		if (comp > 0.0f)
			return;

		comp = fabs(comp);

		if (comp < closestDist)
		{
			closestDist = comp;
			closestEdgeIndex = i;
		}
	}

	auto& closestP0 = facePoints[closestEdgeIndex];
	auto& closestP1 = facePoints[(closestEdgeIndex + 1) % size];

	auto& coll = *m_DataPool->Recycle();

	auto cross = Vector3::Cross(closestP1 - closestP0, edgeP1 - edgeP0);
	auto mag = cross.Magnitude();

	if (mag > 0.00001f)
	{
		cross /= mag;
		coll.Normal1To2 = TransformDirection(cross);
		coll.Penetration = Vector3::Dot(edgeP0 - closestP0, cross);
	}
	else
	{
		auto bodyToBody = m_Shape1->GetTransform().GetPosition() - m_Shape2->GetTransform().GetPosition();
		auto edgeDir = Vector3::Normalize(TransformDirection(edgeP1 - edgeP0));

		coll.Normal1To2 = Vector3::Normalize(Vector3::ProjectOnPlane(edgeDir, bodyToBody));
		coll.Penetration = Vector3::Dot(edgeP0 - closestP0, InverseTransformDirection(coll.Normal1To2));
	}

	coll.Position = TransformPoint(intPoint);
	
	m_FoundCollisions.emplace_back(&coll);
}

void CollisionDetector::FindEdgeCollisions(Shape& shapeFaces, const Vector3& edgeP0, const Vector3& edgeP1)
{
	auto& normals = shapeFaces.GetCachedFaceNormals();
	auto& P0s = shapeFaces.GetCachedFaceP0s();

	for (auto i = 0U; i < P0s.size(); i++)
	{
		auto& P0 = P0s[i];
		auto& normal = normals[i];

		auto comp0 = Vector3::Dot(edgeP0 - P0, normal);
		auto comp1 = Vector3::Dot(edgeP1 - P0, normal);

		if (comp0 > 0.0f && comp1 > 0.0f)
			return;

		if (comp0 * comp1 <= 0.0f)
			FindEdgeCollision(shapeFaces.GetFaces()[i]->GetCachedPoints(), normal, edgeP0, edgeP1);
	}
}

void CollisionDetector::FindEdgeCollisions(Shape& shapeFaces, ArrayWrapper<Vector3, Constants::MaxNumPoints>&  points, const std::vector<int>& edges)
{
	auto pointsData = points.GetData();
	auto size = edges.size();

	for (auto i = 0U; i < size; i += 2)
	{
		auto& edgeP0 = pointsData[edges[i]];
		auto& edgeP1 = pointsData[edges[i + 1]];

		FindEdgeCollisions(shapeFaces, edgeP0, edgeP1);
	}
}

static Vector3 AverageCollPoint(const std::vector<CollisionData*>& collData)
{
	auto ave = Vector3::Zero();

	for (auto it = collData.begin(); it != collData.end(); it++)
		ave += (*it)->Position;

	return ave / static_cast<float>(collData.size());
}

void CollisionDetector::FinalisePointFaceCollisions(Shape& shapeFaces)
{
	if (m_FoundCollisions.size() > 0)
	{
		auto collPoint = AverageCollPoint(m_FoundCollisions);

		auto& faceP0s = shapeFaces.GetCachedFaceP0s();
		auto& faceNormals = shapeFaces.GetCachedFaceNormals();

		auto itN = faceNormals.begin();
		auto itP = faceP0s.begin();

		auto closestDist = MathUtils::Infinity;
		auto indexOfClosest = 0;
		auto currIndex = 0;

		for (; itN != faceNormals.end(); itN++, itP++)
		{
			auto absComp = fabs(Vector3::Dot(collPoint - *itP, *itN));

			if (absComp < closestDist)
			{ 
				closestDist = absComp;
				indexOfClosest = currIndex;
			}

			currIndex++;
		}

		auto& coll = *m_DataPool->Recycle();

		coll.Normal1To2 = TransformDirection(faceNormals[indexOfClosest]);
		coll.Penetration = closestDist;
		coll.Position = TransformPoint(collPoint);

		m_FinalisedFoundCollisions.emplace_back(&coll);
		m_FoundCollisions.clear();
	}
}

// Coll normal will point from body 1 to body 2
void CollisionDetector::FinaliseEdgeCollisions()
{
	if (m_FoundCollisions.size() > 0)
	{
		auto body1To2 = Vector3::Normalize(m_Shape2->GetTransform().GetPosition() - m_Shape1->GetTransform().GetPosition());
		auto maxComp = -1.0f;

		auto& collData = *m_DataPool->Recycle();

		collData.Position = Vector3::Zero();

		for (auto it = m_FoundCollisions.begin(); it != m_FoundCollisions.end(); it++)
		{
			auto data = *it;

			auto comp = fabs(Vector3::Dot(body1To2, data->Normal1To2));

			if (comp > maxComp)
			{
				collData.Normal1To2 = data->Normal1To2;
				collData.Penetration = data->Penetration;
				maxComp = comp;
			}
			collData.Position += data->Position;
		}

		collData.Position /= static_cast<float>(m_FoundCollisions.size());

		m_FinalisedFoundCollisions.emplace_back(&collData);
	}
}

CollisionData* CollisionDetector::FinalCollisionData()
{
	if (m_FinalisedFoundCollisions.size() == 0)
		return nullptr;

	auto body1To2 = Vector3::Normalize(m_Shape2->GetTransform().GetPosition() - m_Shape1->GetTransform().GetPosition());
	auto maxComp = -1.0f;
	CollisionData* best = nullptr;

	for (auto it = m_FinalisedFoundCollisions.begin(); it != m_FinalisedFoundCollisions.end(); it++)
	{
		auto data = *it;

		auto comp = fabs(Vector3::Dot(body1To2, data->Normal1To2));

		if (comp > maxComp)
			best = data;
	}

	if (Vector3::Dot(best->Normal1To2, body1To2) < 0.0f)
		best->Normal1To2 *= -1.0f;

	return best;
}

CollisionData * CollisionDetector::FindCollision(Shape& shape1, Shape& shape2)
{
	m_FoundCollisions.clear();
	m_FinalisedFoundCollisions.clear();
	m_DataPool->Reset();

	m_Shape1TransformedPoints.Clear();
	m_Shape2TransformedPoints.Clear();

	m_Shape1 = &shape1;
	m_Shape2 = &shape2;

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

	m_ActiveTransform = &shape1.GetTransform();
	FindPointFaceCollisions(shape1, m_Shape2TransformedPoints);
	FinalisePointFaceCollisions(shape1);

	m_ActiveTransform = &shape2.GetTransform();
	FindPointFaceCollisions(shape2, m_Shape1TransformedPoints);
	FinalisePointFaceCollisions(shape2);

	FindEdgeCollisions(shape2, m_Shape1TransformedPoints, shape1.GetCachedEdgePoints());
	FinaliseEdgeCollisions();

	return FinalCollisionData();
}