#pragma once
#include <vector>
#include <array>
#include "Vector3.h"

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

class GjkCollisionDetection
{
public:
	struct MinowskiDiffPoint
	{
		Vector3 Value;
		int OriginalA;
		int OriginalB;
		
		MinowskiDiffPoint(const Vector3& val, int origA, int origB) : Value(val), OriginalA(origA), OriginalB(origB)
		{
		}

		MinowskiDiffPoint()
		{
		}
	};

	class SetQ
	{
	public:
		template<class... Targs>
		SetQ(Targs&&... args) : m_Count(sizeof...(args)), m_Points{ std::forward<Targs>(args)... }
		{
		}

		const auto& GetPoints() const
		{
			return m_Points;
		}

		auto GetCount() const
		{
			return m_Count;
		}

		void AddPoint(const MinowskiDiffPoint& p)
		{
			m_Points[m_Count] = p;
			m_Count++;
		}

	private:
		std::array<MinowskiDiffPoint, 4> m_Points;
		int m_Count;
	};

private:
	void InitQ(const GjkInputShape& shapeA, const GjkInputShape& shapeB)
	{
		auto toB = shapeB.GetCentroid() - shapeA.GetCentroid();

		auto origA = shapeA.GetSupportVertex(toB);
		auto origB = shapeB.GetSupportVertex(-toB);

		m_Q = SetQ(MinowskiDiffPoint(shapeA.GetPointAt(origA) - shapeB.GetPointAt(origB), origA, origB));
	}

	bool OriginLiesInEdgesVeroniRegion(const Vector3& q1, const Vector3& q2, const Vector3& qOther) const
	{
		auto n = Vector3::Cross(q2 - q1, qOther - q1);
		return Vector3::Dot(-q1, Vector3::Cross(q2 - q1, n)) >= 0.0f;
	}

	bool OriginLiesInEdgesVeroniRegion(const Vector3& q1, const Vector3& q2, const Vector3& qOtherA, const Vector3& qOtherB) const
	{
		return OriginLiesInEdgesVeroniRegion(q1, q2, qOtherA) && OriginLiesInEdgesVeroniRegion(q1, q2, qOtherB);
	}

	bool OriginLiesInFacesVeroniRegion(const Vector3& q1, const Vector3& q2, const Vector3& q3, const Vector3& qOther) const
	{
		auto n = Vector3::Cross(q2 - q1, q3 - q1);
		return Vector3::Dot(-q1, n) * Vector3::Dot(qOther - q1, n) < 0.0f;
	}

	bool OriginLiesInPointsVeroniRegion(const Vector3& q1, const Vector3& qOther) const
	{
		return Vector3::Dot(-q1, qOther - q1) <= 0.0f;
	}

	bool OriginLiesInPointsVeroniRegion(const Vector3& q1, const Vector3& qOtherA, const Vector3& qOtherB) const
	{
		return OriginLiesInPointsVeroniRegion(q1, qOtherA) && OriginLiesInPointsVeroniRegion(q1, qOtherB);
	}

	bool OriginLiesInPointsVeroniRegion(const Vector3& q1, const Vector3& qOtherA, const Vector3& qOtherB, const Vector3& qOtherC) const
	{
		return OriginLiesInPointsVeroniRegion(q1, qOtherA) && OriginLiesInPointsVeroniRegion(q1, qOtherB) &&
			OriginLiesInPointsVeroniRegion(q1, qOtherC);
	}

	bool TestVertexVeroniRegionsForP(Vector3& p, SetQ& newQ) const
	{
		auto& points = m_Q.GetPoints();
		switch (m_Q.GetCount())
		{
		case 1:
		{
			p = points[0].Value;
			newQ = SetQ(points[0]);
			return true;
		}
		case 2:
		{
			for (auto i = 0; i < 2; i++)
			{
				if (OriginLiesInPointsVeroniRegion(points[i].Value, points[(i + 1) % 2].Value))
				{
					p = points[i].Value;
					newQ = SetQ(points[i]);
					return true;
				}
			}
			return false;
		}
		case 3:
		{
			for (auto i = 0; i < 3; i++)
			{
				if (OriginLiesInPointsVeroniRegion(points[i].Value, points[(i + 1) % 3].Value, points[(i + 2) % 3].Value))
				{
					p = points[i].Value;
					newQ = SetQ(points[i]);
					return true;
				}
			}
			return false;
		}
		case 4:
		{
			for (auto i = 0; i < 4; i++)
			{
				if (OriginLiesInPointsVeroniRegion(points[i].Value, points[(i + 1) % 4].Value, points[(i + 2) % 4].Value, points[(i + 3) % 4].Value))
				{
					p = points[i].Value;
					newQ = SetQ(points[i]);
					return true;
				}
			}
			return false;
		}
		default:
			break;
		}
		assert(false);
		return false;
	}

	Vector3 ProjectOntoEdge(const Vector3& q1, const Vector3& q2) const
	{
		auto d = (q2 - q1).Normalized();
		return q1 + Vector3::Dot(-q1, d) * d;
	}

	bool TestEdgeVeroniRegionsForP(Vector3& p, SetQ& newQ) const
	{
		auto& points = m_Q.GetPoints();
		switch (m_Q.GetCount())
		{
		case 2:
		{
			p = ProjectOntoEdge(points[0].Value, points[1].Value);
			newQ = SetQ(points[0], points[1]);
			return true;
		}
		case 3:
		{
			for (auto i = 0; i < 3; i++)
			{
				auto& pA = points[i];
				auto& pB = points[(i + 1) % 3];
				if (OriginLiesInEdgesVeroniRegion(pA.Value, pB.Value, points[(i + 2) % 3].Value))
				{
					p = ProjectOntoEdge(pA.Value, pB.Value);
					newQ = SetQ(pA, pB);
					return true;
				}
			}
			return false;
		}
		case 4:
		{
			static int toOtherIndexA[] = { 2, 1, 1, 0, 0, 0 };
			static int toOtherIndexB[] = { 3, 3, 2, 3, 2, 1 };

			auto k = 0;
			for (auto i = 0; i < 4; i++)
			{
				for (auto j = i + 1; j < 4; j++)
				{
					if (OriginLiesInEdgesVeroniRegion(points[i].Value, points[j].Value, points[toOtherIndexA[k]].Value, points[toOtherIndexB[k]].Value))
					{
						p = ProjectOntoEdge(points[i].Value, points[j].Value);
						newQ = SetQ(points[i], points[j]);
						return true;
					}
					k++;
				}
			}
			return false;
		}
		default:
			break;
		}
		assert(false);
		return false;
	}

	Vector3 ProjectOntoFace(const Vector3& q1, const Vector3& q2, const Vector3& q3) const
	{
		auto n = Vector3::Cross(q2 - q1, q3 - q1).Normalized();
		return Vector3::Dot(q1, n) * n;
	}

	bool TestFaceVeroniRegionsForP(Vector3& p, SetQ& newQ) const
	{
		auto& points = m_Q.GetPoints();
		switch (m_Q.GetCount())
		{
		case 3:
		{
			p = ProjectOntoFace(points[0].Value, points[1].Value, points[2].Value);
			newQ = SetQ(points[0], points[1], points[2]);
			return true;
		}
		case 4:
		{
			for (auto i = 0; i < 4; i++)
			{
				auto& pA = points[i];
				auto& pB = points[(i + 1) % 4];
				auto& pC = points[(i + 2) % 4];
				if (OriginLiesInFacesVeroniRegion(pA.Value, pB.Value, pC.Value, points[(i + 3) % 4].Value))
				{
					p = ProjectOntoFace(pA.Value, pB.Value, pC.Value);
					newQ = SetQ(pA, pB, pC);
					return true;
				}
			}
			return false;
		}
		default:
			break;
		}
		assert(false);
		return false;
	}

	bool UpdateQ(Vector3& pointOfMinNorm)
	{
		if (TestVertexVeroniRegionsForP(pointOfMinNorm, m_Q))
			return false;

		if (TestEdgeVeroniRegionsForP(pointOfMinNorm, m_Q))
			return false;

		if (TestFaceVeroniRegionsForP(pointOfMinNorm, m_Q))
			return false;

		pointOfMinNorm = Vector3::Zero();
		return true;
	}

public:
	static MinowskiDiffPoint GetMinowskiDiffSupportVertex(const GjkInputShape& shapeA, const GjkInputShape& shapeB, const Vector3& dir)
	{
		auto origA = shapeA.GetSupportVertex(dir);
		auto origB = shapeB.GetSupportVertex(-dir);
		return MinowskiDiffPoint(shapeA.GetPointAt(origA) - shapeB.GetPointAt(origB), origA, origB);
	}

	const auto& GetQ() const
	{
		return m_Q;
	}

	bool Run(const GjkInputShape& shapeA, const GjkInputShape& shapeB)
	{
		InitQ(shapeA, shapeB);

		Vector3 p;
		while (true)
		{
			if (UpdateQ(p))
				return true; // Intersecting

			auto searchDir = -p;
			auto v = GetMinowskiDiffSupportVertex(shapeA, shapeB, searchDir);

			if (Vector3::Dot(v.Value, searchDir) <= 0.0f)
				return false;

			m_Q.AddPoint(v);
		}
	}

private:
	SetQ m_Q;
};
