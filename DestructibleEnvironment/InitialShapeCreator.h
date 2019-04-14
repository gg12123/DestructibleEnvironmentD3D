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
	void Create(CompoundShape& shape, const ShapeProxy& proxy)
	{
		for (auto& ssData : proxy.GetSubShapeData())
		{
			auto ex = ssData.Size / 2.0f;

			auto M = Matrix4::FromTranslation(ssData.Centre) * Matrix4::FromScale(ex.x, ex.y, ex.z);

			auto& subShape = ShapePool::Take();
			m_FacesCreator.CreateFaces(subShape, M, Quaternion::Identity());
			subShape.CollectShapeElementsAndResetHashes();

			shape.AddSubShape(subShape);
		}

		auto& shapes = shape.GetSubShapes();
		for (auto& link : proxy.GetSubShapeLinks())
			shapes[link.ShapeAIndex]->AddLink(*shapes[link.ShapeBIndex]);
	}

private:
	CubeFacesCreator m_FacesCreator;
};
