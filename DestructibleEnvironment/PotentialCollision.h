#pragma once
#include "Vector3.h"
#include "Transform.h"
#include "Constants.h"
#include "Face.h"
#include "UpdatableBound.h"
#include "Shape.h"

class PotentialCollision
{
private:
	UpdatableBound<float> CalculateFacePointsBounds(const Vector3& n) const
	{
		UpdatableBound<float> b;

		auto& points = m_PiercedFace->GetCachedPoints();
		for (auto it = points.begin(); it != points.end(); it++)
			b.Update(Vector3::Dot(*it, n));

		return b;
	}

	UpdatableBound<float> CalculateEdgePointsBounds(const Vector3& n) const
	{
		UpdatableBound<float> b;

		b.Update(Vector3::Dot(m_PiercingEdgeP0, n));
		b.Update(Vector3::Dot(m_PiercingEdgeP1, n));

		return b;
	}

public:
	// the edge is in the faces space
	PotentialCollision(const Vector3& edgeP0, const Vector3& edgeP1, const Vector3& collPointWorld, const Face& piercedFace)
	{
		m_PiercingEdgeP0 = edgeP0;
		m_PiercingEdgeP1 = edgeP1;
		m_CollPointWorld = collPointWorld;
		m_PiercedFace = &piercedFace;
	}

	float CalculateRequiredSeperationWhenMovingEdge(const Vector3& collNormalWorld) const
	{
		auto nLocal = GetTransformOfPiercedFace().ToLocalDirection(collNormalWorld);

		auto f = CalculateFacePointsBounds(nLocal);
		auto e = CalculateEdgePointsBounds(nLocal);

		return f.GetMax() - e.GetMin();
	}

	float CalculateRequiredSeperationWhenMovingFace(const Vector3& collNormalWorld) const
	{
		auto nLocal = GetTransformOfPiercedFace().ToLocalDirection(collNormalWorld);

		auto f = CalculateFacePointsBounds(nLocal);
		auto e = CalculateEdgePointsBounds(nLocal);

		return e.GetMax() - f.GetMin();
	}

	Vector3 GetWorldCollNormal() const
	{
		auto& t = GetTransformOfPiercedFace();
		return t.ToWorldDirection(m_PiercedFace->GetNormal());
	}

	const Vector3& GetCollPointWorld() const
	{
		return m_CollPointWorld;
	}

	// the responder can use to to work out what direction to put the normal in etc.
	Transform& GetTransformOfPiercedFace() const
	{
		return m_PiercedFace->GetShape().GetTransform();
	}

private:
	// in pierced faces space
	Vector3 m_PiercingEdgeP0;
	Vector3 m_PiercingEdgeP1;

	Vector3 m_CollPointWorld;

	const Face* m_PiercedFace;
};
