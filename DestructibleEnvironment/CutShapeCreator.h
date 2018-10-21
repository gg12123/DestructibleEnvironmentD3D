#pragma once
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"
#include "Matrix.h"
#include "CubeFacesCreator.h"

class CutShapeCreator
{
public:
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

	}

	Shape m_CutShape;
	CubeFacesCreator m_FacesCreator;
};