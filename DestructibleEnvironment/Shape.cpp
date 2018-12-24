#include "pch.h"
#include "Shape.h"
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "Face.h"
#include "Random.h"
#include "IntersectionFinder.h"
#include "SmallEdgeRemover.h"

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

void Shape::RemoveSmallEdges()
{
	static SmallEdgeRemover edgeRemover;
	static constexpr float minEdgeLength = 0.001f;
	static constexpr float minEdgeLengthSqr = minEdgeLength * minEdgeLength;

	for (auto itFace = m_Faces.begin(); itFace != m_Faces.end();)
	{
		auto& edges = (*itFace)->GetEdgeObjects();
		auto removed = false;

		for (auto itEdge = edges.begin(); itEdge != edges.end(); itEdge++)
		{
			auto& e = **itEdge;

			if ((e.GetP0().GetPoint() - e.GetP1().GetPoint()).MagnitudeSqr() <= minEdgeLengthSqr)
			{
				auto f1 = *itFace;
				auto f2 = &e.GetOther(*f1);

				edgeRemover.RemoveEdge(e);

				itFace = m_Faces.erase(itFace);

				if (*itFace == f2)
					itFace = m_Faces.erase(itFace);
				else
					m_Faces.erase(std::find(itFace, m_Faces.end(), f2));

				removed = true;
				break;
			}
		}

		if (!removed)
			itFace++;
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

			assert(edges[i]->IsAttachedTo(face));
		}
	}
}

Vector3 Shape::CalculateCentre()
{
	auto c = Vector3::Zero();

	for (auto it = m_PointObjects.begin(); it != m_PointObjects.end(); it++)
		c += (*it)->GetPoint();

	return c / static_cast<float>(m_PointObjects.size());
}

void Shape::ReCentre(const Vector3& centre)
{
	m_CachedPoints.clear();
	m_LocalBounds.Reset();

	for (auto it = m_PointObjects.begin(); it != m_PointObjects.end(); it++)
	{
		auto& p = **it;
		p.ReCentre(centre);

		auto point = p.GetPoint();
		m_CachedPoints.emplace_back(point);
		m_LocalBounds.Update(point);
	}
}

void Shape::InitFacesPointsEdges()
{
	for (auto it = m_Faces.begin(); it != m_Faces.end(); it++)
		(*it)->OnSplittingFinished(*this); // The face will init its points and edges
}