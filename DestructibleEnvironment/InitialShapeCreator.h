#pragma once
#include "Shape.h"
#include "CompoundShape.h"
#include "ShapeProxy.h"
#include "Vector3.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "CubeFacesCreator.h"

class InitialShapeCreator
{
public:
	void Create(CompoundShape& shape, float width, float height, Transform& transform)
	{
		auto w = width / 2.0f;;
		auto h = height / 2.0f;

		auto M = Matrix4::FromScale(w, h, w);

		auto& subShape = ShapePool::Take();
		m_FacesCreator.CreateFaces(subShape, M, Quaternion::Identity());
		subShape.CollectShapeElementsAndResetHashes();

		shape.AddSubShape(subShape);
		shape.CentreAndCache(transform);
	}

private:
	CubeFacesCreator m_FacesCreator;
};
