#pragma once
#include "Matrix.h"
#include "Vector3.h"

class PerObjectShaderConstants
{
public:

	PerObjectShaderConstants()
	{
	}

	Matrix4 ModelToWorldMatrix;
	Vector3 Colour;
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
