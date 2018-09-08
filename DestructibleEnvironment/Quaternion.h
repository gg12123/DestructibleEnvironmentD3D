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
	}

	void Normalize()
	{
		auto mag = sqrt(x * x + y * y + z * z + r * r);

		x /= mag;
		y /= mag;
		z /= mag;
		r /= mag;
	}

	Vector3 RotateV(const Vector3& v) const;

	Quaternion Conj() const
	{
		return Quaternion(r, -x, -y, -z);
	}

	static inline Quaternion Identity()
	{
		Quaternion q;

		q.r = 1.0f;
		q.x = 0.0f;
		q.y = 0.0f;
		q.z = 0.0f;

		return q;
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

		//q.x *= MathUtils::Sign(z.y - y.z);
		//q.y *= MathUtils::Sign(x.z - z.x);
		//q.z *= MathUtils::Sign(y.x - x.y);

		q.x *= MathUtils::Sign(y.z - z.y);
		q.y *= MathUtils::Sign(z.x - x.z);
		q.z *= MathUtils::Sign(x.y - y.x);

		q.Normalize();

		//auto xOut = q.RotateV(Vector3::Right());
		//auto yOut = q.RotateV(Vector3::Up());
		//auto zOut = q.RotateV(Vector3::Foward());

		return q;
	}
};

inline Quaternion operator*(const Quaternion& lhs, float rhs)
{
	return Quaternion(lhs.r * rhs, lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
}

inline Quaternion operator*(float lhs, const Quaternion& rhs)
{
	return rhs * lhs;
}

inline Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs)
{
	return Quaternion(lhs.r + rhs.r, lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

inline Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs)
{
	Quaternion res;

	res.x = lhs.x * rhs.r + lhs.y * rhs.z - lhs.z * rhs.y + lhs.r * rhs.x;
	res.y = -lhs.x * rhs.z + lhs.y * rhs.r + lhs.z * rhs.x + lhs.r * rhs.y;
	res.z = lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.r + lhs.r * rhs.z;
	res.r = -lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z + lhs.r * rhs.r;

	return res;
}

inline Quaternion operator*(const Quaternion& lhs, const Vector3& rhs)
{
	return lhs * Quaternion(0.0f, rhs.x, rhs.y, rhs.z);
}

inline Quaternion operator*(const Vector3& lhs, const Quaternion& rhs)
{
	return Quaternion(0.0f, lhs.x, lhs.y, lhs.z) * rhs;
}