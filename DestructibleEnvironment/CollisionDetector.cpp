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

bool CollisionDetector::FindEdgeCollision(const std::vector<Vector3>& facePoints, const Vector3& faceNormal, const Vector3& edgeP0, const Vector3& edgeP1)
{
	auto intPoint = Vector3::LinePlaneIntersection(facePoints[0], faceNormal, edgeP0, edgeP1);
	auto size = facePoints.size();

	for (auto i = 0U; i < size; i++)
	{
		auto next = (i + 1U) % size;

		auto& P0 = facePoints[i];
		auto& P1 = facePoints[next];

		auto n = Vector3::Normalize(Vector3::Cross(P1 - P0, faceNormal));

		auto comp = Vector3::Dot(n, intPoint - P0);

		if (comp > 0.0f)
			return false;

		comp = fabs(comp);
	}

	m_PotentialCollisionPool->Recycle().Init(intPoint, faceNormal, *m_CurrentOthersPoints, *m_ActiveTransform);
	return true;
}

void CollisionDetector::FindEdgeCollisions(Shape& shapeFaces, const Vector3& edgeP0, const Vector3& edgeP1)
{
	auto& normals = shapeFaces.GetCachedFaceNormals();
	auto& P0s = shapeFaces.GetCachedFaceP0s();

	auto count = 0;

	for (auto i = 0U; i < P0s.size(); i++)
	{
		auto& P0 = P0s[i];
		auto& normal = normals[i];

		auto comp0 = Vector3::Dot(edgeP0 - P0, normal);
		auto comp1 = Vector3::Dot(edgeP1 - P0, normal);

		if (comp0 > 0.0f && comp1 > 0.0f)
			return;

		if (comp0 == 0.0f && comp1 == 0.0f)
			return;

		if (comp0 * comp1 <= 0.0f)
		{
			if (FindEdgeCollision(shapeFaces.GetFaces()[i]->GetCachedPoints(), normal, edgeP0, edgeP1))
				count++;
		}
	}

	if (count == 2)
	{
		auto end = m_PotentialCollisionPool->NumRecycled();
		
		auto& col1 = m_PotentialCollisionPool->At(end - 1);
		auto& col2 = m_PotentialCollisionPool->At(end - 2);

		auto& n1 = col1.GetNormalLocal();
		auto& n2 = col2.GetNormalLocal();

		if (Vector3::Cross(n1, n2).Magnitude() > MathUtils::SmallNumber)
		{
			Vector3 lineP0, lineDir;
			Vector3::LineDefinedByTwoPlanes(col1.GetPointLocal(), n1, col2.GetPointLocal(), n2, lineP0, lineDir);

			auto otherLineDir = (edgeP1 - edgeP0).Normalized();
			auto cross = Vector3::Cross(otherLineDir, lineDir);

			if (cross.Magnitude() > MathUtils::SmallNumber)
			{
				auto normal = cross.InDirectionOf(col1.GetNormalLocal()).Normalized();
				auto pointOnLine = Vector3::PointClosestToOtherLine(lineP0, lineDir, edgeP0, otherLineDir);

				m_PotentialCollisionPool->Recycle().Init(pointOnLine, normal, *m_CurrentOthersPoints, *m_ActiveTransform);
			}
		}
	}
}

void CollisionDetector::FindEdgeCollisions(Shape& shapeFaces, const std::vector<int>& edges)
{
	auto pointsData = m_CurrentOthersPoints->GetData();
	auto size = edges.size();

	for (auto i = 0U; i < size; i += 2)
	{
		auto& edgeP0 = pointsData[edges[i]];
		auto& edgeP1 = pointsData[edges[i + 1]];

		// TODO - when theres a partition, get the faces that may be colliding with this edge

		FindEdgeCollisions(shapeFaces, edgeP0, edgeP1);
	}
}

bool CollisionDetector::FindBestPotentialCollision(CollisionData& outputData)
{
	if (m_PotentialCollisionPool->NumRecycled() == 0)
		return false;

	PotentialCollision *best = nullptr;
	auto highestComp = -1.0f;
	auto dir1To2 = Vector3::Normalize(m_Shape2->GetTransform().GetPosition() - m_Shape1->GetTransform().GetPosition());
	auto& points = outputData.Points;

	points.clear();

	for (auto it = m_PotentialCollisionPool->Begin(); it != m_PotentialCollisionPool->End(); it++)
	{
		auto& coll = *it;

		points.emplace_back(coll.GetPointWorldSpace());

		auto comp = fabs(Vector3::Dot(dir1To2, coll.GetNormalWorldSpace()));

		if (comp > highestComp)
		{
			highestComp = comp;
			best = &coll;
		}
	}

	outputData.Normal1To2 = best->GetNormalWorldSpace().InDirectionOf(dir1To2);
	outputData.Penetration = best->CalculateRequiredSeperation();

	return true;
}

bool CollisionDetector::FindCollision(Shape& shape1, Shape& shape2, CollisionData& outputData)
{
	m_PotentialCollisionPool->Reset();

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
	m_CurrentOthersPoints = &m_Shape2TransformedPoints;
	FindEdgeCollisions(shape1, shape2.GetCachedEdgePoints());

	m_ActiveTransform = &shape2.GetTransform();
	m_CurrentOthersPoints = &m_Shape1TransformedPoints;
	FindEdgeCollisions(shape2, shape1.GetCachedEdgePoints());

	return FindBestPotentialCollision(outputData);
}