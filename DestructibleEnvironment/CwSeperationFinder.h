#pragma once
#include <vector>
#include "Vector3.h"
#include "Matrix.h"
#include "Debug.h"

class CwInputShape
{
public:
	CwInputShape(const SimdStdVector<Vector3>& points) : m_Points(&points)
	{
	}

protected:
	Vector3 FindSupportPointLocal(const Vector3& dir) const
	{
		auto maxComp = MathU::NegativeInfinity;
		const Vector3* support = nullptr;

		for (auto& p : *m_Points)
		{
			auto comp = Vector3::Dot(p, dir);
			if (comp > maxComp)
			{
				maxComp = comp;
				support = &p;
			}
		}
		return *support;
	}

private:
	const SimdStdVector<Vector3>* m_Points;
};

class CwInputShapeA : CwInputShape
{
public:
	CwInputShapeA(const SimdStdVector<Vector3>& points) :
		CwInputShape(points)
	{
	}

	Vector3 FindSupportPoint(const Vector3& dir) const
	{
		return FindSupportPointLocal(dir);
	}
};

class CwInputShapeB : CwInputShape
{
public:
	CwInputShapeB(const SimdStdVector<Vector3>& points, const Quaternion& toB, const Matrix4& toA) :
		CwInputShape(points),
		m_ToASpace(toA),
		m_ToBSpace(toB)
	{
	}

	Vector3 FindSupportPoint(const Vector3& dir) const
	{
		return m_ToASpace * FindSupportPointLocal(m_ToBSpace.RotateV(dir));
	}

private:
	Quaternion m_ToBSpace;
	Matrix4 m_ToASpace;
};

class CwSeperationFinder
{
public:
	bool FindSeperation(const CwInputShapeA& shapeA, const CwInputShapeB& shapeB, Vector3& sepDir)
	{
		static constexpr int maxNumIts = 4;

		for (auto i = 0; i < maxNumIts; i++)
		{
			auto supportA = shapeA.FindSupportPoint(sepDir);
			auto supportB = shapeB.FindSupportPoint(-sepDir);

			if (Vector3::Dot(supportB - supportA, sepDir) > 0.0f)
				return true;

			auto r = (supportB - supportA).Normalized();
			sepDir = sepDir - 2.0f * Vector3::Dot(r, sepDir) * r;
		}
		return false;
	}
};
