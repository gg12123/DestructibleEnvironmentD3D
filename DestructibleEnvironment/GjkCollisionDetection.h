#pragma once
#include <vector>
#include <array>
#include "Vector3.h"

class GjkInputShape
{
public:
	Vector3 GetSupportVertex(const Vector3& dir) const
	{

	}

	Vector3 GetCentroid() const
	{

	}
};

class GjkCollisionDetection
{
private:
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

		void AddPoint(const Vector3& p)
		{
			m_Points[m_Count] = p;
			m_Count++;
		}

	private:
		std::array<Vector3, 4> m_Points;
		int m_Count;
	};

	void InitQ(const GjkInputShape& shapeA, const GjkInputShape& shapeB)
	{

	}

	// P is the point of min norm in the set Q

	bool TestVertexVeroniRegionsForP(Vector3& p, SetQ& newQ)
	{

	}

	bool TestEdgeVeroniRegionsForP(Vector3& p, SetQ& newQ)
	{

	}

	bool TestFaceVeroniRegionsForP(Vector3& p, SetQ& newQ)
	{

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
	float Run(const GjkInputShape& shapeA, const GjkInputShape& shapeB)
	{
		InitQ(shapeA, shapeB);

		Vector3 p;
		while (true)
		{
			if (UpdateQ(p))
				return -1.0f; // Intersecting

			auto pDir = p.Normalized();
			auto v = shapeA.GetSupportVertex(-pDir) - shapeB.GetSupportVertex(pDir);

			// If p is more (or same) extreme in the direction -p, there is no intersection
			if (Vector3::Dot(p, -pDir) >= Vector3::Dot(v, -pDir))
				return p.Magnitude();

			m_Q.AddPoint(v);
		}
	}

private:
	SetQ m_Q;
};
