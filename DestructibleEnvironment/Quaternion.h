#pragma once
#include "Vector3.h"
#include "MathUtils.h"
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
		auto x = Vector3::Cross(y, z);

		Quaternion q;

		q.r = sqrt(MathUtils::Max(0.0f, 1.0f + x.x + y.y + z.z)) / 2.0f;
		q.x = sqrt(MathUtils::Max(0.0f, 1.0f + x.x - y.y - z.z)) / 2.0f;
		q.y = sqrt(MathUtils::Max(0.0f, 1.0f - x.x + y.y - z.z)) / 2.0f;
		q.z = sqrt(MathUtils::Max(0.0f, 1.0f - x.x - y.y + z.z)) / 2.0f;

		q.x *= MathUtils::Sign(z.y - y.z);
		q.y *= MathUtils::Sign(x.z - z.x);
		q.z *= MathUtils::Sign(y.x - x.y);

		return q;
	}
};
