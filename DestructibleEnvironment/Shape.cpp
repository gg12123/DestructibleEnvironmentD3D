#include "pch.h"
#include "Shape.h"
#include "Point.h"
#include "ShapeEdge.h"
#include "Face.h"
#include "Random.h"

void Shape::AddPoint(Point& p)
{
	p.SetId(m_CurrId);
	m_CurrId++;
	m_Points.push_back(&p);
}

Vector3 Shape::CalculateCentre()
{
	auto c = Vector3::Zero();

	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
		c += (*it)->GetPoint();

	return c / static_cast<float>(m_Points.size());
}

Vector3 Shape::CentreAndCache()
{
	auto c = CalculateCentre();

	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
		(*it)->CentreAndCache(c, m_CachedPoints);

	for (auto it = m_Edges.begin(); it != m_Edges.end(); it++)
		(*it)->Cache(m_CachedEdgePoints);

	return c;
}

void Shape::InitFaces(const Vector3& finalFaceNormal)
{
	m_Faces.push_back(&m_FinalFaceCreator.CreateFace(finalFaceNormal));

	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
		(*it)->CachePoints();
}

bool Shape::SplitPoints(const Vector3& P0, const Vector3& n, Shape& shapeAbove, Shape& shapeBelow)
{
	auto numInside = 0;

	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
		(*it)->Split(P0, n, m_NewPointsGetter, shapeAbove, shapeBelow, numInside);

	if (numInside >= 3)
	{
		//Debug.Log(numInside.ToString() + " points inside plane");

		for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
		{
			if ((*it)->CountNumInside() >= 3)
			{
				//Debug.LogWarning("Failed to split shape!");

				// will need to do something to stop the newly created points from leaking here.
				// could you existing inside points to get the new ones out of NewPointsGetter, then delete them.

				return false;
			}
		}
	}
	return true;
}

Vector3 Shape::CalculateSplitPlaneNormal(const Vector3& P0, const Vector3& collNormal)
{
	auto p = Random::Range(0.0f, 0.5f) * m_CachedPoints[Random::Range(0, m_CachedPoints.size())];

	auto toP0 = Vector3::Normalize(P0 - p);

	return Vector3::ProjectOnPlane(Vector3(-toP0.y, -toP0.z, toP0.x), toP0);
}

bool Shape::Split(const Vector3& collPointWs, const Vector3& collNormalWs, Shape& shapeAbove)
{
	auto P0 = m_Transform.ToLocalPosition(collPointWs);
	auto collNormalLocal = m_Transform.ToLocalDirection(collNormalWs);

	auto n = CalculateSplitPlaneNormal(P0, collNormalLocal);

	auto& shapeBelow = *(new Shape()); // from pool

	shapeAbove.Clear();
	shapeBelow.Clear(); // will new clearing when it come from the pool

	if (SplitPoints(P0, n, shapeAbove, shapeBelow))
	{
		for (auto it = m_Edges.begin(); it != m_Edges.end(); it++)
			(*it)->Split(P0, n, m_NewPointsGetter, shapeAbove, shapeBelow);

		for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
			(*it)->Split(m_NewPointsGetter, shapeAbove, shapeBelow);

		m_Points.swap(shapeBelow.GetPoints());
		m_Edges.swap(shapeBelow.GetEdges());
		m_Faces.swap(shapeBelow.GetFaces());

		m_CachedPoints.clear();
		m_CachedEdgePoints.clear();

		InitNewShape(shapeAbove, -n);
		InitNewShape(*this, n);

		// return shape below to pool

		return true;
	}

	return false;
}

void Shape::InitNewShape(Shape& shape, const Vector3& finalFaceNormal)
{
	auto c = shape.CentreAndCache();

	shape.InitFaces(finalFaceNormal);

	shape.GetTransform().SetPosition(m_Transform.ToWorldPosition(c));
	shape.GetTransform().SetRotation(m_Transform.GetRotation());
}