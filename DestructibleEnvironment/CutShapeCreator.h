#pragma once
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"

class CutShapeCreator
{
public:
	// split point and normal must be in the ref transforms space
	Shape & Create(Transform& refTransform, const Vector3& splitPoint, const Vector3& splitNormal)
	{
		auto& shape = *(new Shape()); // from pool
		auto& shapeTran = shape.GetTransform();

		shapeTran.SetEqualTo(refTransform);

		// now position all the faces with centre and rotation defined by split point and normal
	}
};