#include "pch.h"
#include "Shape.h"
#include "Point.h"
#include "ShapeEdge.h"
#include "Face.h"
#include "Random.h"

NewPointsGetter Shape::m_NewPointsGetter;

Shape::~Shape()
{
	// return points, edges, faces to pool.
}

void Shape::AddPoint(Point& p)
{
	p.SetId(m_CurrId);
	m_CurrId++;
	m_Points.emplace_back(&p);
}

void Shape::AddEdge(ShapeEdge& e)
{
	m_Edges.emplace_back(&e);
	m_TotalEdgeLength += e.RegisterLengthWithPoints();
}

Vector3 Shape::CalculateCentre()
{
	auto c = Vector3::Zero();

	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
		c += (*it)->GetPointWeighted();

	return c / m_TotalEdgeLength;
}

Vector3 Shape::CentreAndCache()
{
	auto c = CalculateCentre();

	m_LocalBounds.Reset();
	m_BoundingRadius = MathUtils::NegativeInfinity;

	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
	{
		auto Pcentred = (*it)->CentreAndCache(c, m_CachedPoints);

		auto mag = Pcentred.Magnitude();
		if (mag > m_BoundingRadius)
			m_BoundingRadius = mag;

		m_LocalBounds.Update(Pcentred);
	}

	for (auto it = m_Edges.begin(); it != m_Edges.end(); it++)
		(*it)->Cache(m_CachedEdgePoints);

	return c;
}

void Shape::InitRequiredVertAndIndexCounts()
{
	m_RequiredNumVerts = 0;
	m_RequiredNumIndicies = 0;

	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
	{
		auto numVerts = (*it)->GetPoints().size();

		m_RequiredNumVerts += numVerts;
		m_RequiredNumIndicies += 3 * (numVerts - 2);
	}
}

void Shape::InitFaces(const Vector3& finalFaceNormal)
{
	m_Faces.push_back(&m_FinalFaceCreator.CreateFace(finalFaceNormal));

	m_RequiredNumVerts = 0;
	m_RequiredNumIndicies = 0;

	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
	{
		auto f = *it;

		f->CachePoints(m_CachedFaceNormals, m_CachedFaceP0s);

		auto numVerts = f->GetPoints().size();

		m_RequiredNumVerts += numVerts;
		m_RequiredNumIndicies += 3 * (numVerts - 2);
	}
}

void Shape::TransferSplitResultsToThis(Shape& splitResult)
{
	m_Points.swap(splitResult.m_Points);
	m_Edges.swap(splitResult.m_Edges);
	m_Faces.swap(splitResult.GetFaces());
	m_FinalFaceCreator = splitResult.m_FinalFaceCreator;
	m_TotalEdgeLength = splitResult.m_TotalEdgeLength;

	m_CachedPoints.clear();
	m_CachedEdgePoints.clear();
	m_CachedFaceNormals.clear();
	m_CachedFaceP0s.clear();
}

bool Shape::SplitPoints(const Vector3& P0, const Vector3& n, Shape& shapeAbove, Shape& shapeBelow)
{
	auto numInside = 0;

	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
		(*it)->Split(P0, n, m_NewPointsGetter, shapeAbove, shapeBelow, numInside);

	if (numInside >= 3)
	{
		for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
		{
			if ((*it)->CountNumInside() >= 3)
			{
				// will need to do something to stop the newly created points from leaking here.
				// could you existing inside points to get the new ones out of NewPointsGetter, then delete them.

				return false;
			}
		}
	}
	return true;
}

Vector3 Shape::CalculateSplitPlaneNormal(const Vector3& P0)
{
	auto p = Random::Range(0.0f, 0.5f) * m_CachedPoints[Random::Range(0, m_CachedPoints.size())];

	auto toP0 = Vector3::Normalize(P0 - p);

	return Vector3::ProjectOnPlane(Vector3(-toP0.y, -toP0.z, toP0.x), toP0).Normalized();
}

bool Shape::Split(const Vector3& collPointWs, Shape& shapeAbove)
{
	auto P0 = m_Transform.ToLocalPosition(collPointWs);
	auto n = CalculateSplitPlaneNormal(P0);

	auto& shapeBelow = *(new Shape()); // from pool

	shapeAbove.Clear();
	shapeBelow.Clear(); // will need clearing when it come from the pool

	if (SplitPoints(P0, n, shapeAbove, shapeBelow))
	{
		for (auto it = m_Edges.begin(); it != m_Edges.end(); it++)
			(*it)->Split(P0, n, m_NewPointsGetter, shapeAbove, shapeBelow);

		for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
			(*it)->Split(m_NewPointsGetter, shapeAbove, shapeBelow);

		TransferSplitResultsToThis(shapeBelow);

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

	shape.SetDirty();
}