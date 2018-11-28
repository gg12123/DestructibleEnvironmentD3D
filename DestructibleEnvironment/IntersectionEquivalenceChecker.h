#pragma once
#include "EdgeFaceIntersection.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "Vector3.h"
#include "MathU.h"
#include "ShapePoint.h"

class IntersectionEquivalenceChecker
{
private:
	bool FacesAreConnected(const Face& f1, const Face& f2) const
	{

	}

	bool MakesPotentialEdgeEdgeIntersection(const EdgeFaceIntersection& inter1, const EdgeFaceIntersection& inter2) const
	{
		auto& e1 = inter1.GetEdge();
		auto& e2 = inter2.GetEdge();

		if (&e1 == &e2)
		{
			auto dir = e1.GetDirFromP0ToP1();
			return Vector3::Dot(dir, inter1.GetFace().GetNormal()) * Vector3::Dot(dir, inter2.GetFace().GetNormal()) > 0.0f;
		}
		return false;
	}

	bool ImpliesPointIsOutside(const ShapePoint& p, const EdgeFaceIntersection& inter) const
	{
		auto& e = inter.GetEdge();
		auto pEqualsP0 = &p == &e.GetP0();

		auto dirToPoint = pEqualsP0 ? -e.GetDirFromP0ToP1() : e.GetDirFromP0ToP1();

		return Vector3::Dot(dirToPoint, inter.GetFace().GetNormal()) > 0.0f;
	}

	bool ImplyContradictoryInformationAboutPoint(const EdgeFaceIntersection& inter1, const EdgeFaceIntersection& inter2) const
	{
		auto& e1 = inter1.GetEdge();
		auto& e2 = inter2.GetEdge();

		auto conn = e1.GetConnection(e2);
		if (conn)
			return ImpliesPointIsOutside(*conn, inter1) != ImpliesPointIsOutside(*conn, inter2);

		return false;
	}

public:
	bool AreEquivalent(const EdgeFaceIntersection& inter1, const EdgeFaceIntersection& inter2) const
	{
		if (MakesPotentialEdgeEdgeIntersection(inter1, inter2) || ImplyContradictoryInformationAboutPoint(inter1, inter2))
			return FacesAreConnected(inter1.GetFace(), inter2.GetFace()); // TODO - may also need to check if the faces obscure eachother with respect to the edge directions.

		return false;
	}
};
