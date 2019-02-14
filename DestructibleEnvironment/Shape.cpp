#include "pch.h"
#include "Shape.h"
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "Face.h"
#include "Random.h"
#include "IntersectionFinder.h"
#include "SmallEdgeRemover.h"
#include "FaceTriangulator.h"
#include "ShapeElementPool.h"

Shape::~Shape()
{
	// TODO - return everything to pool.
}

bool Shape::IntersectsRay(const Ray& localRay, Vector3& intPoint)
{
	Vector3 p;
	auto closestHitDist = MathU::Infinity;
	auto hit = false;

	for (auto f : m_Faces)
	{
		if (localRay.IntersectsPlane(f->GetPlaneP0(), f->GetNormal(), p) &&
			f->PointIsOnFace(p))
		{
			auto dist = (p - localRay.GetOrigin()).MagnitudeSqr();
			if (dist < closestHitDist)
			{
				closestHitDist = dist;
				intPoint = p;
				hit = true;
			}
		}
	}

	return hit;
}

void Shape::TryCollectPoint(ShapePoint& p)
{
	if (!p.HasBeenCollected())
	{
		m_PointObjects.emplace_back(&p);
		p.SetIndexInShape(m_PointObjects.size() - 1);
		p.SetBeenCollected();
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

		e.SetBeenCollected();
	}
}

void Shape::CollectShapeElementsAndResetHashes()
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

			edges[i]->ResetHash();
			points[i]->ResetHash();
		}
		face.ResetHash();
	}
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

void Shape::InitFacesPointsEdges()
{
	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
		(*it)->OnSplittingFinished(*this); // The face will init its points and edges
}