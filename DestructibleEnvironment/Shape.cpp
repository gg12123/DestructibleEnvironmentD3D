#include "pch.h"
#include "Shape.h"
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "Face.h"
#include "Random.h"
#include "IntersectionFinder.h"

Shape::~Shape()
{
	// return faces to pool.
}

void Shape::TryCollectPoint(ShapePoint& p)
{
	if (!p.HasBeenCollected())
	{
		m_PointObjects.emplace_back(&p);
		p.SetIndexInShape(m_PointObjects.size() - 1);
		p.SetBeenCollected(true);
	}
}

void Shape::TryCollectEdge(ShapeEdge& e)
{
	// assume that the points are already collected

	if (!e.HasBeenCollected())
	{
		m_EdgeObjects.emplace_back(&e);

		m_EdgeIndexes.emplace_back(e.GetP0().GetIndexInShape());
		m_EdgeIndexes.emplace_back(e.GetP1().GetIndexInShape());

		e.SetBeenCollected(true);
	}
}

void Shape::CollectPointsAndEdges()
{
	m_EdgeObjects.clear();
	m_EdgeIndexes.clear();
	m_PointObjects.clear();

	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
	{
		auto& face = **it;

		auto& points = face.GetPointObjects();
		auto& edges = face.GetEdgeObjects();

		for (auto i = 0U; i < points.size(); i++)
		{
			auto nextI = (i + 1U) % points.size();

			TryCollectPoint(*points[i]);
			TryCollectPoint(*points[nextI]);

			TryCollectEdge(*edges[i]);
		}
	}
}

Vector3 Shape::CalculateCentre()
{
	auto c = Vector3::Zero();

	for (auto it = m_PointObjects.begin(); it != m_PointObjects.end(); it++)
		c += (*it)->GetPoint();

	return c;
}

Vector3 Shape::CalculateSplitPlaneNormal(const Vector3& P0)
{
	auto index = Random::Range(0, m_CachedPoints.size());
	auto p = Random::Range(0.0f, 0.5f) * m_CachedPoints[index];

	auto toP0 = Vector3::Normalize(P0 - p);

	return Vector3::ProjectOnPlane(toP0, Vector3(-toP0.y, -toP0.z, toP0.x)).Normalized();
}

void Shape::ReCentre(const Vector3& centre)
{
	m_CachedPoints.clear();

	for (auto it = m_PointObjects.begin(); it != m_PointObjects.end(); it++)
	{
		auto& p = **it;
		p.ReCentre(centre);
		m_CachedPoints.emplace_back(p.GetPoint());
	}
}

void Shape::InitFaces()
{
	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
		(*it)->OnSplittingFinished(*this);
}