#pragma once
#include <vector>
#include <array>
#include <assert.h>
#include "Vector3.h"
#include "Debug.h"

class GjkInputShape
{
public:
	GjkInputShape(const std::vector<Vector3>& points, const Vector3& centroid) : m_Points(&points), m_Centroid(centroid)
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
	const std::vector<Vector3>* m_Points;
	Vector3 m_Centroid;
};

//class GjkCollisionDetection
//{
//public:
//	struct MinowskiDiffPoint
//	{
//		Vector3 Value;
//		int OriginalA;
//		int OriginalB;
//		
//		MinowskiDiffPoint(const Vector3& val, int origA, int origB) : Value(val), OriginalA(origA), OriginalB(origB)
//		{
//		}
//
//		MinowskiDiffPoint()
//		{
//		}
//	};
//
//	class SetQ
//	{
//	public:
//		template<class... Targs>
//		SetQ(Targs&&... args) : m_Count(sizeof...(args)), m_Points{ std::forward<Targs>(args)... }
//		{
//		}
//
//		const auto& GetPoints() const
//		{
//			return m_Points;
//		}
//
//		auto GetCount() const
//		{
//			return m_Count;
//		}
//
//		void AddPoint(const MinowskiDiffPoint& p)
//		{
//			m_Points[m_Count] = p;
//			m_Count++;
//		}
//
//	private:
//		std::array<MinowskiDiffPoint, 4> m_Points;
//		int m_Count;
//	};
//
//private:
//	void InitQ(const GjkInputShape& shapeA, const GjkInputShape& shapeB)
//	{
//		auto toB = shapeB.GetCentroid() - shapeA.GetCentroid();
//
//		auto origA = shapeA.GetSupportVertex(toB);
//		auto origB = shapeB.GetSupportVertex(-toB);
//
//		m_Q = SetQ(MinowskiDiffPoint(shapeA.GetPointAt(origA) - shapeB.GetPointAt(origB), origA, origB));
//	}
//
//	bool OriginLiesInEdgesVeroniRegion(const Vector3& q1, const Vector3& q2, const Vector3& qOther) const
//	{
//		auto n = Vector3::Cross(q2 - q1, qOther - q1);
//		return Vector3::Dot(-q1, Vector3::Cross(q2 - q1, n)) >= 0.0f;
//	}
//
//	bool OriginLiesInEdgesVeroniRegion(const Vector3& q1, const Vector3& q2, const Vector3& qOtherA, const Vector3& qOtherB) const
//	{
//		return OriginLiesInEdgesVeroniRegion(q1, q2, qOtherA) && OriginLiesInEdgesVeroniRegion(q1, q2, qOtherB);
//	}
//
//	bool OriginLiesInFacesVeroniRegion(const Vector3& q1, const Vector3& q2, const Vector3& q3, const Vector3& qOther) const
//	{
//		auto n = Vector3::Cross(q2 - q1, q3 - q1);
//		return Vector3::Dot(-q1, n) * Vector3::Dot(qOther - q1, n) < 0.0f;
//	}
//
//	bool OriginLiesInPointsVeroniRegion(const Vector3& q1, const Vector3& qOther) const
//	{
//		return Vector3::Dot(-q1, qOther - q1) <= 0.0f;
//	}
//
//	bool OriginLiesInPointsVeroniRegion(const Vector3& q1, const Vector3& qOtherA, const Vector3& qOtherB) const
//	{
//		return OriginLiesInPointsVeroniRegion(q1, qOtherA) && OriginLiesInPointsVeroniRegion(q1, qOtherB);
//	}
//
//	bool OriginLiesInPointsVeroniRegion(const Vector3& q1, const Vector3& qOtherA, const Vector3& qOtherB, const Vector3& qOtherC) const
//	{
//		return OriginLiesInPointsVeroniRegion(q1, qOtherA) && OriginLiesInPointsVeroniRegion(q1, qOtherB) &&
//			OriginLiesInPointsVeroniRegion(q1, qOtherC);
//	}
//
//	bool TestVertexVeroniRegionsForP(Vector3& p, SetQ& newQ) const
//	{
//		auto& points = m_Q.GetPoints();
//		switch (m_Q.GetCount())
//		{
//		case 1:
//		{
//			p = points[0].Value;
//			newQ = SetQ(points[0]);
//			return true;
//		}
//		case 2:
//		{
//			for (auto i = 0; i < 2; i++)
//			{
//				if (OriginLiesInPointsVeroniRegion(points[i].Value, points[(i + 1) % 2].Value))
//				{
//					p = points[i].Value;
//					newQ = SetQ(points[i]);
//					return true;
//				}
//			}
//			return false;
//		}
//		case 3:
//		{
//			for (auto i = 0; i < 3; i++)
//			{
//				if (OriginLiesInPointsVeroniRegion(points[i].Value, points[(i + 1) % 3].Value, points[(i + 2) % 3].Value))
//				{
//					p = points[i].Value;
//					newQ = SetQ(points[i]);
//					return true;
//				}
//			}
//			return false;
//		}
//		case 4:
//		{
//			for (auto i = 0; i < 4; i++)
//			{
//				if (OriginLiesInPointsVeroniRegion(points[i].Value, points[(i + 1) % 4].Value, points[(i + 2) % 4].Value, points[(i + 3) % 4].Value))
//				{
//					p = points[i].Value;
//					newQ = SetQ(points[i]);
//					return true;
//				}
//			}
//			return false;
//		}
//		default:
//			break;
//		}
//		assert(false);
//		return false;
//	}
//
//	Vector3 ProjectOntoEdge(const Vector3& q1, const Vector3& q2) const
//	{
//		auto d = (q2 - q1).Normalized();
//		return q1 + Vector3::Dot(-q1, d) * d;
//	}
//
//	bool TestEdgeVeroniRegionsForP(Vector3& p, SetQ& newQ) const
//	{
//		auto& points = m_Q.GetPoints();
//		switch (m_Q.GetCount())
//		{
//		case 2:
//		{
//			p = ProjectOntoEdge(points[0].Value, points[1].Value);
//			newQ = SetQ(points[0], points[1]);
//			return true;
//		}
//		case 3:
//		{
//			for (auto i = 0; i < 3; i++)
//			{
//				auto& pA = points[i];
//				auto& pB = points[(i + 1) % 3];
//				if (OriginLiesInEdgesVeroniRegion(pA.Value, pB.Value, points[(i + 2) % 3].Value))
//				{
//					p = ProjectOntoEdge(pA.Value, pB.Value);
//					newQ = SetQ(pA, pB);
//					return true;
//				}
//			}
//			return false;
//		}
//		case 4:
//		{
//			static int toOtherIndexA[] = { 2, 1, 1, 0, 0, 0 };
//			static int toOtherIndexB[] = { 3, 3, 2, 3, 2, 1 };
//
//			auto k = 0;
//			for (auto i = 0; i < 4; i++)
//			{
//				for (auto j = i + 1; j < 4; j++)
//				{
//					if (OriginLiesInEdgesVeroniRegion(points[i].Value, points[j].Value, points[toOtherIndexA[k]].Value, points[toOtherIndexB[k]].Value))
//					{
//						p = ProjectOntoEdge(points[i].Value, points[j].Value);
//						newQ = SetQ(points[i], points[j]);
//						return true;
//					}
//					k++;
//				}
//			}
//			return false;
//		}
//		default:
//			break;
//		}
//		assert(false);
//		return false;
//	}
//
//	Vector3 ProjectOntoFace(const Vector3& q1, const Vector3& q2, const Vector3& q3) const
//	{
//		auto n = Vector3::Cross(q2 - q1, q3 - q1).Normalized();
//		return Vector3::Dot(q1, n) * n;
//	}
//
//	bool TestFaceVeroniRegionsForP(Vector3& p, SetQ& newQ) const
//	{
//		auto& points = m_Q.GetPoints();
//		switch (m_Q.GetCount())
//		{
//		case 3:
//		{
//			p = ProjectOntoFace(points[0].Value, points[1].Value, points[2].Value);
//			newQ = SetQ(points[0], points[1], points[2]);
//			return true;
//		}
//		case 4:
//		{
//			for (auto i = 0; i < 4; i++)
//			{
//				auto& pA = points[i];
//				auto& pB = points[(i + 1) % 4];
//				auto& pC = points[(i + 2) % 4];
//				if (OriginLiesInFacesVeroniRegion(pA.Value, pB.Value, pC.Value, points[(i + 3) % 4].Value))
//				{
//					p = ProjectOntoFace(pA.Value, pB.Value, pC.Value);
//					newQ = SetQ(pA, pB, pC);
//					return true;
//				}
//			}
//			return false;
//		}
//		default:
//			break;
//		}
//		assert(false);
//		return false;
//	}
//
//	bool UpdateQ(Vector3& pointOfMinNorm)
//	{
//		if (TestVertexVeroniRegionsForP(pointOfMinNorm, m_Q))
//			return false;
//
//		if (TestEdgeVeroniRegionsForP(pointOfMinNorm, m_Q))
//			return false;
//
//		if (TestFaceVeroniRegionsForP(pointOfMinNorm, m_Q))
//			return false;
//
//		pointOfMinNorm = Vector3::Zero();
//		return true;
//	}
//
//public:
//	static MinowskiDiffPoint GetMinowskiDiffSupportVertex(const GjkInputShape& shapeA, const GjkInputShape& shapeB, const Vector3& dir)
//	{
//		auto origA = shapeA.GetSupportVertex(dir);
//		auto origB = shapeB.GetSupportVertex(-dir);
//		return MinowskiDiffPoint(shapeA.GetPointAt(origA) - shapeB.GetPointAt(origB), origA, origB);
//	}
//
//	const auto& GetQ() const
//	{
//		return m_Q;
//	}
//
//	bool Run(const GjkInputShape& shapeA, const GjkInputShape& shapeB)
//	{
//		InitQ(shapeA, shapeB);
//		m_PrevDist = MathU::Infinity;
//
//		Vector3 p;
//		while (true)
//		{
//			if (UpdateQ(p))
//				return true; // Intersecting
//
//			// TODO - why is this termination criteria needed?
//			// Is there something wrong in the implementation?
//			auto dist = p.MagnitudeSqr();
//			if (m_PrevDist <= dist)
//				return false;
//
//			m_PrevDist = dist;
//
//			auto searchDir = -p;
//			auto v = GetMinowskiDiffSupportVertex(shapeA, shapeB, searchDir);
//
//			if (Vector3::Dot(v.Value, searchDir) <= 0.0f)
//				return false;
//
//			m_Q.AddPoint(v);
//		}
//	}
//
//private:
//	SetQ m_Q;
//	float m_PrevDist;
//};


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
	};

	enum class GjkResult
	{
		Intersection,
		NoIntersection,
		Error,
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
		//auto toB = shapeB.GetCentroid() - shapeA.GetCentroid();
		//
		//auto origA = shapeA.GetSupportVertex(toB);
		//auto origB = shapeB.GetSupportVertex(-toB);
		//
		//m_Q = SetQ(MinowskiDiffPoint(shapeA.GetPointAt(origA) - shapeB.GetPointAt(origB), origA, origB));
		auto toB = shapeB.GetCentroid() - shapeA.GetCentroid();
		AddNextPointToSimplex(shapeA, shapeB, toB, simplex);
	}

	void UpdateLineSegmentSimplex(Simplex& simplex, Vector3& pMin, Vector3& searchDir)
	{
		static constexpr int aIndex = 0, bIndex = 1;

		auto& points = simplex.Points;
		auto a = points[aIndex];
		auto b = points[bIndex];

		auto ab = b - a;
		auto denom = Vector3::Dot(ab, ab);

		if (denom <= MathU::Epsilon)
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
		auto a = points[aIndex];
		auto b = points[bIndex];
		auto c = points[cIndex];

		auto ab = b - a;
		auto ac = c - a;
		auto ap = -a;

		auto n = Vector3::Cross(ab, ac);	
		if (n.MagnitudeSqr() <= MathU::Epsilon)
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
			searchDir = Vector3::Cross(ab, n).InDirectionOf(-pMin);
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
			searchDir = Vector3::Cross(ac, n).InDirectionOf(-pMin);
			simplex.RemovePoint(bIndex);
			return;
		}

		// Edge region bc
		auto va = d3 * d6 - d5 * d4;
		if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
		{
			auto w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			auto bc = c - b;
			pMin = b + w * bc;
			searchDir = Vector3::Cross(bc, n).InDirectionOf(-pMin);
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
		auto a = points[aIndex];
		auto b = points[bIndex];
		auto c = points[cIndex];
		auto d = points[dIndex];

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
			if (distSq <= MathU::Epsilon)
				return GjkResultLocal::DegenerateSearchDirection;

			if (distSq >= prvDistSq)
				return GjkResultLocal::NotGettingCloser;

			prvDistSq = distSq;

			AddNextPointToSimplex(shapeA, shapeB, searchDir, simplex);
			auto& v = simplex.Points[simplex.NumPoints - 1];

			if (Vector3::Dot(v, searchDir) <= 0.0f)
				return GjkResultLocal::NoIntersection;
		}
	}

	GjkResult HandleDegenerateSearchDirection(const GjkInputShape& shapeA, const GjkInputShape shapeB, Simplex& simplex)
	{
		switch (simplex.NumPoints)
		{
		case 1:
		{
			// This means that two of the shapes points are touching. Report no intersection for now
			// then on the next tick the shapes will most likely be intersecting properly.
			return GjkResult::NoIntersection;
		}
		case 2:
		{
			Debug::Log(std::string("Gjk search direction degenerated with line segment simplex."));

			// TODO - although unlikely, the shapes could still be intersecting in this case.
			// Need to handle it somehow.

			return GjkResult::NoIntersection;
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
				return GjkResult::NoIntersection;

			auto sMinus = GetSupportPoint(shapeA, shapeB, -n, iaMinus, ibMinus);
			if (MathU::Abs(Vector3::Dot(sMinus - a, -n)) <= tol)
				return GjkResult::NoIntersection;

			if (Vector3::Dot(a, n) > 0.0f)
				simplex.AddPoint(sMinus, iaMinus, ibMinus);
			else
				simplex.AddPoint(sPlus, iaPlus, ibPlus);

			return GjkResult::Intersection;
		}
		default:
			break;
		}
		assert(false);
		return GjkResult::Error;
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
		{
			return HandleDegenerateSearchDirection(shapeA, shapeB, simplex);
		}
		case GjkCollisionDetector::GjkResultLocal::NotGettingCloser:
		{
			// TODO - not sure if this always means no intersection.
			return GjkResult::NoIntersection;
		}
		default:
			break;
		}
		assert(false);
		return GjkResult::Error;
	}
};