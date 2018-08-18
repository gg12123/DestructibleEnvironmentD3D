#pragma once
#include "Matrix.h"
#include "Vector.h"

class PerObjectShaderConstants
{
public:

	PerObjectShaderConstants()
	{
	}

	Matrix4 ModelToWorldMatrix;
};

class PerSceneShaderConstants
{
public:
	PerSceneShaderConstants()
	{
	}

	Matrix4 VPMatrix;
	Vector3 LightPosition;
	Vector3 ViewDirection;
};
