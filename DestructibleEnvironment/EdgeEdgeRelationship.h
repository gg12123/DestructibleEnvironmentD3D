#pragma once
#include "Face.h"
#include "ShapeEdge.h"
#include "MapToPointPlaneRelationship.h"
#include "Vector3.h"

class EdgeEdgeRelationship
{
private:
	enum class EdgeRelationshipWithFace
	{
		Mixed,
		PointsAbove,
		PointsBelow
	};

	bool IsInsideCornerEdge(const ShapeEdge& edge) const
	{
		auto& faceEdge = edge.GetFace1();
		auto& faceNormal = edge.GetFace2();

		auto dot = Vector3::Dot(faceEdge.GetEdgeNormal(edge), faceNormal.GetNormal());
		assert(dot != 0.0f);

		return dot > 0.0f;
	}

	bool IsOutsideCornerEdge(const ShapeEdge& edge) const
	{
		// Assumes the planes are not equal.
		return !IsInsideCornerEdge(edge);
	}

	bool ImpliesIntersection(EdgeRelationshipWithFace sameSideRelationship, const ShapeEdge& edgeFaces) const
	{
		// TODO - this may get un-stable as the two planes become equal.

		if (sameSideRelationship == EdgeRelationshipWithFace::PointsAbove)
		{
			return IsOutsideCornerEdge(edgeFaces);
		}
		else if (sameSideRelationship == EdgeRelationshipWithFace::PointsBelow)
		{
			return IsInsideCornerEdge(edgeFaces);
		}
		assert(false);
		return false;
	}

	bool IntersectionWithPlaneImpliesIntersectionWithFace(const Face& face, const ShapeEdge& edgeOnFace, const ShapeEdge& piercingEdge) const
	{
		Vector3 intPoint;
		assert(Vector3::LinePlaneIntersection(face.GetPlaneP0(), face.GetNormal(), piercingEdge.GetP0().GetPoint(), piercingEdge.GetP1().GetPoint(), intPoint));

		return Vector3::Dot(face.GetEdgeNormal(edgeOnFace), intPoint - edgeOnFace.GetP0().GetPoint()) <= 0.0f;
	}

	EdgeRelationshipWithFace GetEdgeFaceRelationship(const ShapeEdge& edge, const Face& face, const MapToPointPlaneRelationship& map) const
	{
		auto p0Rela = map.GetRelationship(face, edge.GetP0());
		auto p1Rela = map.GetRelationship(face, edge.GetP1());

		if (p0Rela != p1Rela)
			return EdgeRelationshipWithFace::Mixed;

		if (p0Rela == PointPlaneRelationship::PointsAbove)
			return EdgeRelationshipWithFace::PointsAbove;

		return EdgeRelationshipWithFace::PointsBelow;
	}

public:
	EdgeEdgeRelationship()
	{
	}

	EdgeEdgeRelationship(const ShapeEdge& piercingEdge, const ShapeEdge& edgeFaces, const MapToPointPlaneRelationship& mapToPointPlane)
	{
		m_Face1 = &edgeFaces.GetFace1();
		m_Face2 = &edgeFaces.GetFace2();

		auto relationshipWithFace1 = GetEdgeFaceRelationship(piercingEdge, *m_Face1, mapToPointPlane);
		auto relationshipWithFace2 = GetEdgeFaceRelationship(piercingEdge, *m_Face2, mapToPointPlane);

		if (relationshipWithFace1 == EdgeRelationshipWithFace::Mixed && relationshipWithFace2 != EdgeRelationshipWithFace::Mixed)
		{
			m_ImpliesIntersectionForFace1 = ImpliesIntersection(relationshipWithFace2, edgeFaces);
			m_ImpliesIntersectionForFace2 = false;
		}
		else if (relationshipWithFace2 == EdgeRelationshipWithFace::Mixed && relationshipWithFace1 != EdgeRelationshipWithFace::Mixed)
		{
			m_ImpliesIntersectionForFace2 = ImpliesIntersection(relationshipWithFace1, edgeFaces);
			m_ImpliesIntersectionForFace1 = false;
		}
		else if (relationshipWithFace1 == EdgeRelationshipWithFace::Mixed && relationshipWithFace2 == EdgeRelationshipWithFace::Mixed)
		{
			m_ImpliesIntersectionForFace1 = IntersectionWithPlaneImpliesIntersectionWithFace(*m_Face1, edgeFaces, piercingEdge);

			auto& p0 = piercingEdge.GetP0();
			auto intersectionsCanDoubleUp = mapToPointPlane.GetRelationship(edgeFaces.GetFace1(), p0) == mapToPointPlane.GetRelationship(edgeFaces.GetFace2(), p0);

			m_ImpliesIntersectionForFace2 = intersectionsCanDoubleUp ? !m_ImpliesIntersectionForFace1 : m_ImpliesIntersectionForFace1;
		}
		else
		{
			m_ImpliesIntersectionForFace1 = m_ImpliesIntersectionForFace2 = false;
		}
	}

	bool ImpliesIntersectionFor(const Face& face) const
	{
		if (&face == m_Face1)
			return m_ImpliesIntersectionForFace1;
		
		if (&face == m_Face2)
			return m_ImpliesIntersectionForFace2;

		assert(false);
		return false;
	}

private:
	const Face* m_Face1;
	const Face* m_Face2;

	bool m_ImpliesIntersectionForFace1;
	bool m_ImpliesIntersectionForFace2;
};