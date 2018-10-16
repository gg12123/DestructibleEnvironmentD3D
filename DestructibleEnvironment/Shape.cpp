#include "pch.h"
#include "Shape.h"
#include "Point.h"
#include "ShapeEdge.h"
#include "Face.h"
#include "Random.h"
#include "IntersectionFinder.h"

Shape::~Shape()
{
	// return faces to pool.
}

Vector3 Shape::CalculateCentre()
{
	auto c = Vector3::Zero();
	auto count = 0;

	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
	{
		auto& points = (*it)->GetCachedPoints();

		for (auto it = points.begin(); it != points.end(); it++)
		{
			// TODO - use the edge length weighting. Maybe calculate a weight for each point when the point gets added to the face
			c += (*it); 
			count++;
		}
	}
	return c / count;
}

void Shape::InitRequiredVertAndIndexCounts()
{
	m_RequiredNumVerts = 0;
	m_RequiredNumIndicies = 0;

	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
	{
		auto numVerts = (*it)->GetCachedPoints().size();

		m_RequiredNumVerts += numVerts;
		m_RequiredNumIndicies += 3 * (numVerts - 2);
	}
}

Vector3 Shape::CalculateSplitPlaneNormal(const Vector3& P0)
{
	auto index = Random::Range(0, m_CachedPoints.size());
	auto p = Random::Range(0.0f, 0.5f) * m_CachedPoints[index];

	auto toP0 = Vector3::Normalize(P0 - p);

	return Vector3::ProjectOnPlane(toP0, Vector3(-toP0.y, -toP0.z, toP0.x)).Normalized();
}

void Shape::ReCentreFaces(const Vector3& centre)
{
	// this is also when the faces regster their points so clear cached points now
	m_CachedPoints.clear();

	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
		(*it)->ReCentre(centre, *this);
}