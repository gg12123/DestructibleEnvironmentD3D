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

	struct DWithChosenFace
	{
		const Face * const FBc;
		const Vector3 D;

		DWithChosenFace(const Face& fBc, const Vector3& d) :
			FBc(&fBc),
			D(d)
		{
		}
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

	bool TryCalculateD(const Face& fBi, const Face& fA0, Vector3& d) const
	{
		d = Vector3::Cross(fBi.GetNormal(), fA0.GetNormal());
		auto mag = d.Magnitude();
		if (mag > 0.0f)
		{
			d /= mag;
			return true;
		}
		return false;
	}

	DWithChosenFace CalculateD(const ShapeEdge& eB, const Face& fA0, const Vector3& eAN) const
	{
		auto& fB0 = eB.GetFace1();
		auto& fB1 = eB.GetFace2();

		Vector3 temp;

		auto d0 = TryCalculateD(fB0, fA0, temp) ?
			temp.InDirectionOf(-fB0.GetEdgeNormal(eB)) :
			-fB0.GetEdgeNormal(eB);

		auto d1 = TryCalculateD(fB1, fA0, temp) ?
			temp.InDirectionOf(-fB1.GetEdgeNormal(eB)) :
			-fB1.GetEdgeNormal(eB);

		//auto d0 = AssertDExists(fB0, fA0).InDirectionOf(-fB0.GetEdgeNormal(eB));
		//auto d1 = AssertDExists(fB1, fA0).InDirectionOf(-fB1.GetEdgeNormal(eB));;

		return MathU::Abs(Vector3::Dot(d0, eAN)) > MathU::Abs(Vector3::Dot(d1, eAN)) ?
			DWithChosenFace(fB0, d0) :
			DWithChosenFace(fB1, d1);
	}

	bool FBcIntersectionImplied(const DWithChosenFace& d, const Vector3& eAN, bool X) const
	{
		auto dot = Vector3::Dot(d.D, eAN);
		assert(dot != 0.0f);

		if ((dot < 0.0f && X) || (dot > 0.0f && !X))
			return false;

		return true;
	}

public:
	EdgeEdgeRelationship() = default;

	EdgeEdgeRelationship(const ShapeEdge& edgeFaces,
		const ShapeEdge& piercingEdge,
		const Face& bridgedByEdgeFaces,
		bool intersectionImpliedForBridgedFace,
		const MapToPointPlaneRelationship& mapToPointPlane)
	{
		auto& eB = edgeFaces;
		auto& eA = piercingEdge;

		auto& fA0 = bridgedByEdgeFaces;
		auto& fA1 = eA.GetOther(fA0);

		auto& fB0 = eB.GetFace1();
		auto& fB1 = eB.GetFace2();

		auto relationshipWithFB0 = GetEdgeFaceRelationship(eA, fB0, mapToPointPlane);
		auto relationshipWithFB1 = GetEdgeFaceRelationship(eA, fB1, mapToPointPlane);

		if (relationshipWithFB0 == EdgeRelationshipWithFace::Mixed && relationshipWithFB1 == EdgeRelationshipWithFace::Mixed)
		{
			auto eAN = fA0.GetEdgeNormal(eA);
			auto d = CalculateD(eB, fA0, eAN);

			auto& fBc = *d.FBc;
			auto& fBOther = eB.GetOther(fBc);

			auto intersectionImpliedForFBc = FBcIntersectionImplied(d, eAN, intersectionImpliedForBridgedFace);

			auto& pA0 = eA.GetP0();

			auto intersectionImpliedForFBOther = mapToPointPlane.GetRelationship(fB0, pA0) == mapToPointPlane.GetRelationship(fB1, pA0) ?
				!intersectionImpliedForFBc :
				intersectionImpliedForFBc;

			m_Face1 = &fBc;
			m_ImpliesIntersectionForFace1 = intersectionImpliedForFBc;

			m_Face2 = &fBOther;
			m_ImpliesIntersectionForFace2 = intersectionImpliedForFBOther;
		}
		else
		{
			assert(relationshipWithFB0 != EdgeRelationshipWithFace::Mixed && relationshipWithFB1 != EdgeRelationshipWithFace::Mixed);

			m_Face1 = &eB.GetFace1();
			m_Face2 = &eB.GetFace2();

			m_ImpliesIntersectionForFace1 = false;
			m_ImpliesIntersectionForFace2 = false;
		}

		m_IsConstrained = false;
		m_PiercingEdge = &eA;
	}

	EdgeEdgeRelationship(const ShapeEdge& piercingEdge, const ShapeEdge& edgeFaces, const MapToPointPlaneRelationship& mapToPointPlane)
	{
		m_Face1 = &edgeFaces.GetFace1();
		m_Face2 = &edgeFaces.GetFace2();
		m_PiercingEdge = &piercingEdge;

		auto relationshipWithFace1 = GetEdgeFaceRelationship(piercingEdge, *m_Face1, mapToPointPlane);
		auto relationshipWithFace2 = GetEdgeFaceRelationship(piercingEdge, *m_Face2, mapToPointPlane);

		if (relationshipWithFace1 == EdgeRelationshipWithFace::Mixed && relationshipWithFace2 != EdgeRelationshipWithFace::Mixed)
		{
			m_ImpliesIntersectionForFace1 = ImpliesIntersection(relationshipWithFace2, edgeFaces);
			m_ImpliesIntersectionForFace2 = false;
			m_IsConstrained = true;
		}
		else if (relationshipWithFace2 == EdgeRelationshipWithFace::Mixed && relationshipWithFace1 != EdgeRelationshipWithFace::Mixed)
		{
			m_ImpliesIntersectionForFace2 = ImpliesIntersection(relationshipWithFace1, edgeFaces);
			m_ImpliesIntersectionForFace1 = false;
			m_IsConstrained = true;
		}
		else if (relationshipWithFace1 == EdgeRelationshipWithFace::Mixed && relationshipWithFace2 == EdgeRelationshipWithFace::Mixed)
		{
			m_IsConstrained = false;

			m_ImpliesIntersectionForFace1 = IntersectionWithPlaneImpliesIntersectionWithFace(*m_Face1, edgeFaces, piercingEdge);

			auto& p0 = piercingEdge.GetP0();
			auto intersectionsCanDoubleUp = mapToPointPlane.GetRelationship(edgeFaces.GetFace1(), p0) == mapToPointPlane.GetRelationship(edgeFaces.GetFace2(), p0);

			m_ImpliesIntersectionForFace2 = intersectionsCanDoubleUp ? !m_ImpliesIntersectionForFace1 : m_ImpliesIntersectionForFace1;
		}
		else
		{
			m_IsConstrained = false;
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

	const Face* GetBridgedFace(const MapToPointPlaneRelationship& mapToPointPlane) const
	{
		auto r1 = GetEdgeFaceRelationship(*m_PiercingEdge, *m_Face1, mapToPointPlane);
		if (r1 == EdgeRelationshipWithFace::Mixed)
			return m_Face1;

		auto r2 = GetEdgeFaceRelationship(*m_PiercingEdge, *m_Face2, mapToPointPlane);
		if (r2 == EdgeRelationshipWithFace::Mixed)
			return m_Face2;

		return nullptr;
	}

	bool IsConstrained() const
	{
		return m_IsConstrained;
	}

private:
	const Face* m_Face1;
	const Face* m_Face2;

	bool m_ImpliesIntersectionForFace1;
	bool m_ImpliesIntersectionForFace2;

	const ShapeEdge* m_PiercingEdge;

	bool m_IsConstrained;
};

static_assert(std::is_trivial<EdgeEdgeRelationship>::value, "Edge edge relationship must be trivial.");