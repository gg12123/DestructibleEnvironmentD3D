#pragma once
#include "Vector.h"

class Vertex
{
public:
	Vertex(const Vector3& pos, const Vector3& normal)
	{
		Position = pos;
		Normal = normal;
	}

	Vector3 Position;
	Vector3 Normal;
};
