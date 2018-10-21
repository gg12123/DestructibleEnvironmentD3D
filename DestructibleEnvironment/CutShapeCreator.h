#pragma once
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"
#include "Matrix.h"
#include "CubeFacesCreator.h"
#include "Random.h"

class CutShapeCreator
{
public:
	CutShapeCreator()
	{

	}

	// split point and normal must be in the to split transforms space
	Shape & Create(Transform& toSplitsTransform, const Vector3& splitPoint, const Vector3& splitNormal)
	{
		m_CutShape.Clear();

		auto& shapeTran = m_CutShape.GetTransform();
		shapeTran.SetEqualTo(toSplitsTransform);

		auto M = CalculateCutShapesTransform(toSplitsTransform, splitPoint, splitNormal);
		m_FacesCreator.CreateFaces(m_CutShape, M);

		for (int i = 0; i < 8; i++)
			m_CutShape.AddPoint(m_FacesCreator.GetSharedPoint(i, M));

		return m_CutShape;
	}

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

	Shape m_CutShape;
	CubeFacesCreator m_FacesCreator;
};