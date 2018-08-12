#pragma once
#include "Matrix.h"
#include "Vector.h"

class PerObjectShaderConstants
{
public:
	Matrix4 ModelToWorldMatrix;
};

class PerSceneShaderConstants
{
public:
	Matrix4 VPMatrix;
	Vector3 LightPosition;
	Vector3 ViewDirection;
};
