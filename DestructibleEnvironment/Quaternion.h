#pragma once
#include "Vector3.h"
#include <assert.h>

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
		assert(false);
		return Vector3::Zero();
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
		assert(fabs(Vector3::Dot(forward, up) < 0.9f));

		auto z = forward;
		auto y = Vector3::Normalize(Vector3::ProjectOnPlane(forward, up));
		auto x = Vector3::Cross(y, z); // TODO - is this the right order?

		Quaternion res;

		res.r = sqrt(1.0f + x.x + y.y + z.z) * 0.5f;

		auto xx = 1.0f / (4.0f * res.r);

		res.x = (z.y - y.z) * xx;
		res.y = (x.z - z.x) * xx;
		res.z = (y.x - x.y) * xx;

		return res;
	}
};
