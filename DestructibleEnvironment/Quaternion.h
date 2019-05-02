#pragma once
#include "Vector3.h"
#include "MathU.h"
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

	static inline Quaternion AngleAxis(float angle, const Vector3& axis)
	{
		auto halfCosAngle = 0.5f * cos(angle);
		auto halfSinAngle = 0.5f * sin(angle);

		return Quaternion(halfCosAngle, halfSinAngle * axis.X(), halfSinAngle * axis.Y(), halfSinAngle * axis.Z());
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

		q.r = sqrt(MathU::Max(0.0f, 1.0f + x.X() + y.Y() + z.Z())) / 2.0f;
		q.x = sqrt(MathU::Max(0.0f, 1.0f + x.X() - y.Y() - z.Z())) / 2.0f;
		q.y = sqrt(MathU::Max(0.0f, 1.0f - x.X() + y.Y() - z.Z())) / 2.0f;
		q.z = sqrt(MathU::Max(0.0f, 1.0f - x.X() - y.Y() + z.Z())) / 2.0f;

		q.x *= MathU::Sign(y.Z() - z.Y());
		q.y *= MathU::Sign(z.X() - x.Z());
		q.z *= MathU::Sign(x.Y() - y.X());

		q.Normalize();

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
	return lhs * Quaternion(0.0f, rhs.X(), rhs.Y(), rhs.Z());
}

inline Quaternion operator*(const Vector3& lhs, const Quaternion& rhs)
{
	return Quaternion(0.0f, lhs.X(), lhs.Y(), lhs.Z()) * rhs;
}