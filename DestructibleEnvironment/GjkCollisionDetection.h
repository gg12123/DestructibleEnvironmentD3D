#pragma once
#include <vector>
#include <array>
#include <assert.h>
#include "Vector3.h"
#include "Debug.h"

class GjkInputShape
{
public:
	GjkInputShape(const SimdStdVector<Vector3>& points, const Vector3& centroid) : m_Points(&points), m_Centroid(centroid)
	{
	}

	int GetSupportVertex(const Vector3& dir) const
	{
		auto& points = *m_Points;
		auto supportsIndex = 0u;
		auto maxComp = MathU::NegativeInfinity;

		for (auto i = 0u; i < points.size(); i++)
		{
			auto comp = Vector3::Dot(points[i], dir);
			if (comp > maxComp)
			{
				maxComp = comp;
				supportsIndex = i;
			}
		}
		return supportsIndex;
	}

	Vector3 GetCentroid() const
	{
		return m_Centroid;
	}

	const auto& GetPointAt(int index) const
	{
		return (*m_Points)[index];
	}

private:
	const SimdStdVector<Vector3>* m_Points;
	Vector3 m_Centroid;
};

class GjkCollisionDetector
{
public:
	class Simplex
	{
	public:
		std::array<Vector3, 4> Points;
		std::array<int, 4> IndexesA;
		std::array<int, 4> IndexesB;
		int NumPoints;

		Simplex() : NumPoints(0)
		{
		}

		void RemovePoint(int index)
		{
			NumPoints--;
			Points[index] = Points[NumPoints];
			IndexesA[index] = IndexesA[NumPoints];
			IndexesB[index] = IndexesB[NumPoints];
		}

		void AddPoint(const Vector3& p, int iA, int iB)
		{
			Points[NumPoints] = p;
			IndexesA[NumPoints] = iA;
			IndexesB[NumPoints] = iB;
			NumPoints++;
		}

		void ReCalculatePoints(const GjkInputShape& shapeA, const GjkInputShape shapeB)
		{
			for (auto i = 0; i < NumPoints; i++)
			{
				Points[i] = shapeA.GetPointAt(IndexesA[i]) - shapeB.GetPointAt(IndexesB[i]);
			}
		}
	};

	enum class GjkResult
	{
		Intersection,
		MaybeIntersection,
		NoIntersection
	};

private:
	enum class GjkResultLocal
	{
		Intersection,
		NoIntersection,
		DegenerateSearchDirection,
		NotGettingCloser
	};

	void InitSimplex(const GjkInputShape& shapeA, const GjkInputShape shapeB, Simplex& simplex)
	{
		auto toB = shapeB.GetCentroid() - shapeA.GetCentroid();
		AddNextPointToSimplex(shapeA, shapeB, toB, simplex);
	}

	void UpdateLineSegmentSimplex(Simplex& simplex, Vector3& pMin, Vector3& searchDir)
	{
		static constexpr int aIndex = 0, bIndex = 1;

		auto& points = simplex.Points;
		auto& a = points[aIndex];
		auto& b = points[bIndex];

		auto ab = b - a;
		auto denom = Vector3::Dot(ab, ab);

		if (denom <= MathU::EpsilonSqr)
		{
			// Degenerate line segment - Could pick a or b.
			pMin = a;
			searchDir = -a;
			simplex.RemovePoint(bIndex);
			return;
		}

		auto t = Vector3::Dot(-a, ab) / denom;

		if (t < 0.0f)
		{
			pMin = a;
			searchDir = -a;
			simplex.RemovePoint(bIndex);
			return;
		}

		if (t > 1.0f)
		{
			pMin = b;
			searchDir = -b;
			simplex.RemovePoint(aIndex);
			return;
		}

		pMin = a + t * ab;
		searchDir = -pMin;
	}

	void HandleDegenerateTriangleSimplex(Simplex& simplex, Vector3& pMin, Vector3& searchDir)
	{
		auto closestDist = MathU::Infinity;
		Simplex simplexOfClosest;

		for (auto i = 0; i < 3; i++)
		{
			auto simplexTemp = simplex;
			simplexTemp.RemovePoint(i);

			Vector3 pMinTemp, serchDirTemp;
			UpdateLineSegmentSimplex(simplexTemp, pMinTemp, serchDirTemp);

			auto mag = pMinTemp.MagnitudeSqr();
			if (mag < closestDist)
			{
				closestDist = mag;
				simplexOfClosest = simplexTemp;
				pMin = pMinTemp;
				searchDir = serchDirTemp;
			}
		}
		simplex = simplexOfClosest;
	}

	void UpdateTriangleSimplex(Simplex& simplex, Vector3& pMin, Vector3& searchDir)
	{
		static constexpr int aIndex = 0, bIndex = 1, cIndex = 2;

		auto& points = simplex.Points;
		auto& a = points[aIndex];
		auto& b = points[bIndex];
		auto& c = points[cIndex];

		auto ab = b - a;
		auto ac = c - a;
		auto ap = -a;

		auto n = Vector3::Cross(ab, ac);
		if (n.MagnitudeSqr() <= MathU::EpsilonSqr)
		{
			HandleDegenerateTriangleSimplex(simplex, pMin, searchDir);
			return;
		}

		// Vertex region a
		auto d1 = Vector3::Dot(ab, ap);
		auto d2 = Vector3::Dot(ac, ap);
		if (d1 <= 0.0f && d2 <= 0.0f)
		{
			pMin = a;
			searchDir = -a;
			simplex.RemovePoint(bIndex);
			simplex.RemovePoint(cIndex);
			return;
		}

		// Vertex region b
		auto bp = -b;
		auto d3 = Vector3::Dot(ab, bp);
		auto d4 = Vector3::Dot(ac, bp);
		if (d3 >= 0.0f && d4 <= d3)
		{
			pMin = b;
			searchDir = -b;
			simplex.RemovePoint(aIndex);
			simplex.RemovePoint(cIndex);
			return;
		}

		// Edge region ab
		auto vc = d1 * d4 - d3 * d2;
		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
		{
			auto v = d1 / (d1 - d3);
			pMin = a + v * ab;
			searchDir = -pMin;
			simplex.RemovePoint(cIndex);
			return;
		}

		// Vertex region c
		auto cp = -c;
		auto d5 = Vector3::Dot(ab, cp);
		auto d6 = Vector3::Dot(ac, cp);
		if (d6 >= 0.0f && d5 <= d6)
		{
			pMin = c;
			searchDir = -c;
			simplex.RemovePoint(aIndex);
			simplex.RemovePoint(bIndex);
			return;
		}

		// Edge region ac
		auto vb = d5 * d2 - d1 * d6;
		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
		{
			auto w = d2 / (d2 - d6);
			pMin = a + w * ac;
			searchDir = -pMin;
			simplex.RemovePoint(bIndex);
			return;
		}

		// Edge region bc
		auto va = d3 * d6 - d5 * d4;
		if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
		{
			auto w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			pMin = b + w * (c - b);
			searchDir = -pMin;
			simplex.RemovePoint(aIndex);
			return;
		}

		// Face region
		auto denom = 1.0f / (va + vb + vc);
		auto v = vb * denom;
		auto w = vc * denom;
		pMin = a + ab * v + ac * w;
		searchDir = n.InDirectionOf(-pMin);
	}

	void HandleDegenerateTetrahedronSimplex(Simplex& simplex, Vector3& pMin, Vector3& searchDir)
	{
		auto closestDist = MathU::Infinity;
		Simplex simplexOfClosest;

		for (auto i = 0; i < 4; i++)
		{
			auto simplexTemp = simplex;
			simplexTemp.RemovePoint(i);

			Vector3 pMinTemp, serchDirTemp;
			UpdateTriangleSimplex(simplexTemp, pMinTemp, serchDirTemp);

			auto mag = pMinTemp.MagnitudeSqr();
			if (mag < closestDist)
			{
				closestDist = mag;
				simplexOfClosest = simplexTemp;
				pMin = pMinTemp;
				searchDir = serchDirTemp;
			}
		}
		simplex = simplexOfClosest;
	}

	bool PointOutsideOfPlane(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d)
	{
		auto x = Vector3::Cross(b - a, c - a);
		auto signp = Vector3::Dot(-a, x);
		auto signd = Vector3::Dot(d - a, x);

		return signp * signd <= 0.0f;
	}

	void UpdateTetrahedronSimplex(Simplex& simplex, Vector3& pMin, Vector3& searchDir)
	{
		static constexpr int aIndex = 0, bIndex = 1, cIndex = 2, dIndex = 3;

		auto& points = simplex.Points;
		auto& a = points[aIndex];
		auto& b = points[bIndex];
		auto& c = points[cIndex];
		auto& d = points[dIndex];

		if (MathU::Abs(Vector3::Dot(a - d, Vector3::Cross(b - a, c - a))) <= MathU::Epsilon)
		{
			HandleDegenerateTetrahedronSimplex(simplex, pMin, searchDir);
			return;
		}

		auto closestDist = MathU::Infinity;
		auto simplexOfClosest = simplex;
		Vector3 pMinTemp, searchDirTemp;

		// Test face abc
		if (PointOutsideOfPlane(a, b, c, d))
		{
			auto simplexTemp = simplex;
			simplexTemp.RemovePoint(dIndex);
			UpdateTriangleSimplex(simplexTemp, pMinTemp, searchDirTemp);
			auto dist = pMinTemp.MagnitudeSqr();

			if (dist < closestDist)
			{
				pMin = pMinTemp;
				searchDir = searchDirTemp;
				simplexOfClosest = simplexTemp;
			}
		}

		// Test face acd
		if (PointOutsideOfPlane(a, c, d, b))
		{
			auto simplexTemp = simplex;
			simplexTemp.RemovePoint(bIndex);
			UpdateTriangleSimplex(simplexTemp, pMinTemp, searchDirTemp);
			auto dist = pMinTemp.MagnitudeSqr();

			if (dist < closestDist)
			{
				pMin = pMinTemp;
				searchDir = searchDirTemp;
				simplexOfClosest = simplexTemp;
			}
		}

		// Test face adb
		if (PointOutsideOfPlane(a, d, b, c))
		{
			auto simplexTemp = simplex;
			simplexTemp.RemovePoint(cIndex);
			UpdateTriangleSimplex(simplexTemp, pMinTemp, searchDirTemp);
			auto dist = pMinTemp.MagnitudeSqr();

			if (dist < closestDist)
			{
				pMin = pMinTemp;
				searchDir = searchDirTemp;
				simplexOfClosest = simplexTemp;
			}
		}

		// Test face bdc
		if (PointOutsideOfPlane(b, d, c, a))
		{
			auto simplexTemp = simplex;
			simplexTemp.RemovePoint(aIndex);
			UpdateTriangleSimplex(simplexTemp, pMinTemp, searchDirTemp);
			auto dist = pMinTemp.MagnitudeSqr();

			if (dist < closestDist)
			{
				pMin = pMinTemp;
				searchDir = searchDirTemp;
				simplexOfClosest = simplexTemp;
			}
		}

		simplex = simplexOfClosest;
	}

	void UpdateSimplex(Simplex& simplex, Vector3& pMin, Vector3& searchDir)
	{
		switch (simplex.NumPoints)
		{
		case 1:
		{
			pMin = simplex.Points[0];
			searchDir = -pMin;
			return;
		}
		case 2:
		{
			UpdateLineSegmentSimplex(simplex, pMin, searchDir);
			return;
		}
		case 3:
		{
			UpdateTriangleSimplex(simplex, pMin, searchDir);
			return;
		}
		case 4:
		{
			UpdateTetrahedronSimplex(simplex, pMin, searchDir);
			return;
		}
		default:
			break;
		}
		assert(false);
	}

	void AddNextPointToSimplex(const GjkInputShape& shapeA, const GjkInputShape shapeB, const Vector3& searchDir, Simplex& simplex)
	{
		int iA, iB;
		auto p = GetSupportPoint(shapeA, shapeB, searchDir, iA, iB);
		simplex.AddPoint(p, iA, iB);
	}

	GjkResultLocal RunLocal(const GjkInputShape& shapeA, const GjkInputShape shapeB, Simplex& simplex)
	{
		auto prvDistSq = MathU::Infinity;
		Vector3 pMin, searchDir;

		while (true)
		{
			UpdateSimplex(simplex, pMin, searchDir);

			if (simplex.NumPoints == 4)
				return GjkResultLocal::Intersection;

			auto distSq = pMin.MagnitudeSqr();
			if (distSq <= MathU::EpsilonSqr)
				return GjkResultLocal::DegenerateSearchDirection;

			if (distSq >= prvDistSq)
				return GjkResultLocal::NotGettingCloser;

			prvDistSq = distSq;

			AddNextPointToSimplex(shapeA, shapeB, searchDir, simplex);
			auto& v = simplex.Points[simplex.NumPoints - 1];

			if (Vector3::Dot(v, searchDir) < 0.0f)
				return GjkResultLocal::NoIntersection;
		}
	}

	GjkResult CheckOnSupportFace(const GjkInputShape& shapeA, const GjkInputShape shapeB, const Vector3& n, Simplex& simplex)
	{
		auto& a = simplex.Points[0];

		if (Vector3::Dot(a, n) >= 0.0f)
		{
			// This is a support face and the origin is behind it so this 
			// probably means intersection.

			int iA, iB;
			auto otherPoint = GetSupportPoint(shapeA, shapeB, -n, iA, iB);
			simplex.AddPoint(otherPoint, iA, iB);
			return GjkResult::MaybeIntersection;
		}

		// If the origin is infront of a support face - no intersection.
		return GjkResult::NoIntersection;
	}

	GjkResult CheckSimplex(const GjkInputShape& shapeA, const GjkInputShape shapeB, Simplex& simplex)
	{
		switch (simplex.NumPoints)
		{
		case 1:
		{
			// This could definitly mean that the shapes are point on point touching.
			// Maybe they could also be fully intersecting???

			// For now just report no intersection to save on epa calls.
			// Hopefully this doesnt cause problems!
			return GjkResult::NoIntersection;
		}
		case 2:
		{
			// TODO - the simplex output here is very often degenerate. Needs improvement.
			// Also if the edge in the simplex is a support edge, it may be safe to return
			// no intersection?

			auto& points = simplex.Points;
			auto& a = points[0];
			auto& b = points[1];

			auto t = a - b;
			auto d1 = Vector3::OrthogonalDirection(t);
			auto d2 = Vector3::Cross(d1, t);

			int iA, iB;

			auto sp1 = GetSupportPoint(shapeA, shapeB, d1, iA, iB);
			simplex.AddPoint(sp1, iA, iB);

			auto sp2 = GetSupportPoint(shapeA, shapeB, d2, iA, iB);
			simplex.AddPoint(sp2, iA, iB);

			return GjkResult::MaybeIntersection;
		}
		case 3:
		{
			auto& points = simplex.Points;
			auto& a = points[0];
			auto& b = points[1];
			auto& c = points[2];

			auto n = Vector3::Cross(b - a, c - a);
			int iaPlus, ibPlus;
			int iaMinus, ibMinus;

			static constexpr auto tol = 0.0001f;

			auto sPlus = GetSupportPoint(shapeA, shapeB, n, iaPlus, ibPlus);
			if (MathU::Abs(Vector3::Dot(sPlus - a, n)) <= tol)
				return CheckOnSupportFace(shapeA, shapeB, n, simplex);

			auto minusN = -n;
			auto sMinus = GetSupportPoint(shapeA, shapeB, minusN, iaMinus, ibMinus);
			if (MathU::Abs(Vector3::Dot(sMinus - a, minusN)) <= tol)
				return CheckOnSupportFace(shapeA, shapeB, minusN, simplex);

			// Neither face is a support so this could be an intersection
			if (Vector3::Dot(a, n) > 0.0f)
				simplex.AddPoint(sMinus, iaMinus, ibMinus);
			else
				simplex.AddPoint(sPlus, iaPlus, ibPlus);

			return GjkResult::MaybeIntersection;
		}
		default:
			break;
		}
		assert(false);
		return GjkResult::NoIntersection;
	}

public:
	static Vector3 GetSupportPoint(const GjkInputShape& shapeA, const GjkInputShape shapeB, const Vector3& searchDir, int& iA, int& iB)
	{
		iA = shapeA.GetSupportVertex(searchDir);
		iB = shapeB.GetSupportVertex(-searchDir);
		return shapeA.GetPointAt(iA) - shapeB.GetPointAt(iB);
	}

	GjkResult Run(const GjkInputShape& shapeA, const GjkInputShape shapeB, Simplex& simplex)
	{
		if (simplex.NumPoints == 0)
			InitSimplex(shapeA, shapeB, simplex);

		auto localRes = RunLocal(shapeA, shapeB, simplex);

		switch (localRes)
		{
		case GjkCollisionDetector::GjkResultLocal::Intersection:
		{
			return GjkResult::Intersection;
		}
		case GjkCollisionDetector::GjkResultLocal::NoIntersection:
		{
			return GjkResult::NoIntersection;
		}
		case GjkCollisionDetector::GjkResultLocal::DegenerateSearchDirection:
		case GjkCollisionDetector::GjkResultLocal::NotGettingCloser:
		{
			return CheckSimplex(shapeA, shapeB, simplex);
		}
		default:
			break;
		}
		assert(false);
		return GjkResult::NoIntersection;
	}
};