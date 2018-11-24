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
	bool IntersectionPointsAreEqual(const EdgeFaceIntersection& inter1, const EdgeFaceIntersection& inter2) const
	{
		// TODO - this is not vert robust. A better approach would be to check the distance
		// as travelled along the shape that owns the faces.

		// But the best would not involve an arbitrary tolerance.

		static constexpr float equalTol = 0.00001f;
		return (inter1.GetIntPoint() - inter2.GetIntPoint()).Magnitude() <= equalTol;
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
			return IntersectionPointsAreEqual(inter1, inter2);

		return false;
	}
};
