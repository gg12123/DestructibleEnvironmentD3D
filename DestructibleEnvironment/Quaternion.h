#pragma once
#include "Vector.h"

class Quaternion
{
public:

	float r;

	float x;
	float y;
	float z;

	Quaternion(float rComp, float xComp, float yComp, float zComp)
	{
		r = rComp;
		x = xComp;
		y = yComp;
		z = zComp;
	}

	Quaternion()
	{
		r = 1.0f;
		x = y = z = 0.0f;
	}

	Vector3 Rotate(const Vector3& toRot) const
	{

	}

	Quaternion Conj() const
	{
		return Quaternion(r, -x, -y, -z);
	}

	static inline Quaternion Identity()
	{
		return Quaternion();
	}

	static inline Quaternion LookRotation(const Vector3& forward)
	{
		return LookRotation(forward, Vector3::Up());
	}

	static inline Quaternion LookRotation(const Vector3& forward, const Vector3& up)
	{

	}
};
