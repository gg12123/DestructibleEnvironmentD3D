#pragma once
#include "Entity.h"
#include "Matrix.h"

class Camera : public Entity
{
public:
	Matrix4 CalcuateVPMatrix();

	Vector3 GetViewDirection();
};
