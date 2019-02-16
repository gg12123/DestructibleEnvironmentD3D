#pragma once
#include <vector>
#include "Vector3.h"

class GjkInputShape
{
public:
	Vector3 GetSupportVertex(const Vector3& dir) const
	{

	}

	Vector3 GetCentroid() const
	{

	}
};

class GjkCollisionDetection
{
public:


private:
	std::vector<Vector3> m_Q;
};
