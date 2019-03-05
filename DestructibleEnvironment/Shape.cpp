#include "pch.h"
#include "Shape.h"
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "Face.h"
#include "Random.h"
#include "SmallEdgeRemover.h"
#include "FaceTriangulator.h"
#include "ShapeElementPool.h"
#include "ShapeDuplicater.h"
#include "CompoundShape.h"

bool Shape::IntersectsRay(const Ray& localRay, Vector3& intPoint)
{
	Vector3 p;
	auto closestHitDist = MathU::Infinity;
	auto hit = false;

	for (auto f : m_Faces)
	{
		// TODO - seeing as the shape is now convex, this can be done faster.
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
	// TODO - this could be done without the 'been collected flag'
	// if we assume that each point is attached to three faces.
	// At the moment this assumption holds but after the clean
	// geometry method is implemented it may not. So re-vist this
	// after implementing that.

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

void Shape::ResetBeenCollectedFlag()
{
	for (auto p : m_PointObjects)
		p->ClearBeenCollected();

	for (auto e : m_EdgeObjects)
		e->ClearBeenCollected();
}

static BoundsCalculator BoundsCalc;

void Shape::ReCentre(const Vector3& ownersCentre)
{
	BoundsCalc.Reset();

	m_CachedPoints.clear();

	for (auto it = m_PointObjects.begin(); it != m_PointObjects.end(); it++)
	{
		auto& p = **it;
		p.ReCentre(ownersCentre);

		auto& pp = p.GetPoint();
		m_CachedPoints.emplace_back(pp);
		BoundsCalc.Update(pp);
	}
	m_Centre -= ownersCentre;
	m_LocalAABB = BoundsCalc.ToAABB();
}

void Shape::UpdateWorldAABB()
{
	BoundsCalc.Reset();

	auto c = m_LocalAABB.GetCentre();
	auto e = m_LocalAABB.GetExtends();

	auto& t = m_Owner->GetTransform();

	BoundsCalc.Update(t.ToWorldPosition(c + Vector3(e.x, e.y, e.z)));
	BoundsCalc.Update(t.ToWorldPosition(c + Vector3(-e.x, e.y, e.z)));
	BoundsCalc.Update(t.ToWorldPosition(c + Vector3(-e.x, e.y, -e.z)));
	BoundsCalc.Update(t.ToWorldPosition(c + Vector3(e.x, e.y, -e.z)));

	BoundsCalc.Update(t.ToWorldPosition(c + Vector3(e.x, -e.y, e.z)));
	BoundsCalc.Update(t.ToWorldPosition(c + Vector3(-e.x, -e.y, e.z)));
	BoundsCalc.Update(t.ToWorldPosition(c + Vector3(-e.x, -e.y, -e.z)));
	BoundsCalc.Update(t.ToWorldPosition(c + Vector3(e.x, -e.y, -e.z)));

	m_WorldAABB = BoundsCalc.ToAABB();
}

void Shape::CalculateCentre()
{
	m_Centre = Vector3::Zero();

	for (auto p : m_PointObjects)
		m_Centre += p->GetPoint();

	m_Centre /= static_cast<float>(m_PointObjects.size());
}

void Shape::InitFacesPointsEdges()
{
	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
		(*it)->OnSplittingFinished(*this); // The face will init its points and edges
}

Shape& Shape::Duplicate() const
{
	static ShapeDuplicater duplicator;
	auto& dup = duplicator.Duplicate(*this);
	dup.m_Centre = m_Centre;
	return dup;
}

void Shape::OnReturnedToPool()
{
	for (auto& e : m_EdgeObjects)
		EdgePool::Return(*e);

	for (auto& f : m_Faces)
		FacePool::Return(*f);

	for (auto& p : m_PointObjects)
		PointPool::Return(*p);
}