#pragma once
#include <array>
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"
#include "Matrix.h"
#include "CubeFacesCreator.h"
#include "Random.h"

class CutShapeCreator
{
private:
	Matrix4 CalculateCutShapesTransform(Transform& toSplitsTransform, const Vector3& splitPoint, const Vector3& splitNormal)
	{
		auto sZ = 100.0f;
		auto r = Random::Range(0.0f, 0.5f);

		auto A = r * splitPoint;
		auto C = A + sZ * splitNormal;

		auto q = Quaternion::LookRotation(-splitNormal, Vector3::OrthogonalDirection(splitNormal));

		return Matrix4::FromTranslation(C) * Matrix4::FromRotation(q) * Matrix4::FromScale(sZ, sZ, sZ);
	}

public:
	Shape & Create(Transform& toSplitsTransform, const Vector3& splitPointWorld, const Vector3& splitNormalWorld)
	{
		m_CutShape.Clear();
		m_CutShape.GetTransform().SetEqualTo(toSplitsTransform);

		auto splitPoint = toSplitsTransform.ToLocalPosition(splitPointWorld);
		auto splitNormal = toSplitsTransform.ToLocalDirection(splitNormalWorld);

		auto M = CalculateCutShapesTransform(toSplitsTransform, splitPoint, splitNormal);

		m_FacesCreator.CreateFaces(m_CutShape, M);

		m_CutShape.OnAllFacesAdded();

		return m_CutShape;
	}

private:
	Shape m_CutShape;
	CubeFacesCreator m_FacesCreator;
};