#pragma once
#include "Shape.h"
#include "ShapeProxy.h"
#include "Vector3.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "CubeFacesCreator.h"

class InitialShapeCreator
{
public:
	void Create(Shape& shape, float width, float height, const Transform& transform)
	{
		auto w = width / 2.0f;;
		auto h = height / 2.0f;

		auto M = Matrix4::FromScale(w, h, w);

		shape.Clear();
		m_FacesCreator.CreateFaces(shape, M, Quaternion::Identity());

		shape.GetTransform().SetEqualTo(transform);
		shape.OnAllFacesAdded();
	}

private:
	CubeFacesCreator m_FacesCreator;
};
