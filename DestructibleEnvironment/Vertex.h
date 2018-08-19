#pragma once
#include "Vector3.h"

class Vertex
{
public:
	Vertex()
	{
	}

	Vertex(const Vector3& pos, const Vector3& normal)
	{
		Position = pos;
		Normal = normal;
	}

	Vector3 Position;
	Vector3 Normal;
};
