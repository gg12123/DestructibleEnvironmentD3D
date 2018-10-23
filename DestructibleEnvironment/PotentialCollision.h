#pragma once
#include "Vector3.h"
#include "Transform.h"
#include "ArrayWrapper.h"
#include "Constants.h"
#include "Face.h"

class PotentialCollision
{
public:
	PotentialCollision(const Vector3& edgeP0, const Vector3& edgeP1, const Vector3& collPointWorld, const Face& piercedFace)
	{
		m_PiercingEdgeP0 = edgeP0;
		m_PiercingEdgeP1 = edgeP1;
		m_CollPointWorld = collPointWorld;
		m_PiercedFace = &piercedFace;
	}

	float CalculateRequiredSeperationWhenMovingEdge(const Vector3 collNormalWorld) const
	{

	}

	float CalculateRequiredSeperationWhenMovingFace(const Vector3 collNormalWorld) const
	{

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
