#pragma once
#include <math.h>
#include "MathU.h"

class Vector3
{
public:
	float x;
	float y;
	float z;

	Vector3()
	{
		x = y = z = 0.0f;
	}

	Vector3(float xComp, float yComp, float zComp)
	{
		x = xComp;
		y = yComp;
		z = zComp;
	}

	Vector3 operator-() const
	{
		return Vector3(-x, -y, -z);
	}

	Vector3& operator*=(float rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;
		return *this;
	}

	void operator+=(const Vector3& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
	}

	void operator-=(const Vector3& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
	}

	void operator/=(float rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
	}

	float Magnitude() const
	{
		return sqrt(x * x + y * y + z * z);
	}

	float MagnitudeSqr() const
	{
		return (x * x + y * y + z * z);
	}

	Vector3 InDirectionOf(const Vector3& refDir) const
	{
		auto& thisRef = *this;
		return (Vector3::Dot(thisRef, refDir) < 0.0f) ? -thisRef : thisRef;
	}

	Vector3 Normalized() const
	{
		return Vector3::Normalize(*this);
	}

	void Normalize()
	{
		auto mag = Magnitude();

		if (mag > 0.0f)
			(*this) /= Magnitude();
	}

	// static

	static inline float Dot(const Vector3& v1, const Vector3& v2);
	static inline Vector3 Cross(const Vector3& v1, const Vector3& v2);
	static inline Vector3 ProjectOnPlane(const Vector3& planeNormal, const Vector3& vector);
	static inline bool LinePlaneIntersection(const Vector3& planeP0, const Vector3& planeNormal, const Vector3& lineP0, const Vector3& lineP1, Vector3& intPoint);
	static inline bool FindLineDefinedByTwoPlanes(const Vector3& planeP0, const Vector3& planeN0, const Vector3& planeP1, const Vector3& planeN1, Vector3& lineP0, Vector3& lineDir);
	//static inline Vector3 PointClosestToOtherLine(const Vector3& lineP0, const Vector3& lineDir, const Vector3& otherLineP0, const Vector3& otherLineDir);
	static inline Vector3 Zero();
	static inline Vector3 Normalize(const Vector3& v);
	static inline Vector3 Right();
	static inline Vector3 Up();
	static inline Vector3 Foward();
	static  Vector3 OrthogonalDirection(const Vector3& v);
};

inline Vector3 operator+(const Vector3& lhs, const Vector3& rhs)
{
	return Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

inline Vector3 operator-(const Vector3& lhs, const Vector3& rhs)
{
	return Vector3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

inline Vector3 operator/(const Vector3& lhs, float rhs)
{
	return Vector3(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
}

inline Vector3 operator*(float lhs, const Vector3& rhs)
{
	return Vector3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

inline Vector3 operator*(const Vector3& lhs, float rhs)
{
	return rhs * lhs;
}

inline float Vector3::Dot(const Vector3& v1, const Vector3& v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline Vector3 Vector3::Cross(const Vector3& v1, const Vector3& v2)
{
	Vector3 res;

	res.x = (v1.y * v2.z) - (v1.z * v2.y);
	res.y = -((v1.x * v2.z) - (v1.z * v2.x));
	res.z = (v1.x * v2.y) - (v1.y * v2.x);

	return res;
}

inline Vector3 Vector3::ProjectOnPlane(const Vector3& planeNormal, const Vector3& vector)
{
	return vector - (planeNormal * Dot(vector, planeNormal));
}

// assuming that the line intersect the plane
inline bool Vector3::LinePlaneIntersection(const Vector3& planeP0, const Vector3& planeNormal, const Vector3& lineP0, const Vector3& lineP1, Vector3& intPoint)
{
	auto l = Normalize(lineP1 - lineP0);

	auto num = Vector3::Dot(planeP0 - lineP0, planeNormal);
	auto denom = Vector3::Dot(l, planeNormal);

	if (denom == 0.0f)
		return false;

	auto u = num / denom;

	intPoint = lineP0 + u * l;
	return true;
}

inline Vector3 Vector3::Zero()
{
	return Vector3(0.0f, 0.0f, 0.0f);
}

inline Vector3 Vector3::Normalize(const Vector3& v)
{
	auto res = v;
	auto mag = res.Magnitude();
	return (mag > 0.0f ? res / mag : Vector3::Zero());
}

inline Vector3 Vector3::Right()
{
	return Vector3(1.0f, 0.0f, 0.0f);
}

inline Vector3 Vector3::Up()
{
	return Vector3(0.0f, 1.0f, 0.0f);
}

inline Vector3 Vector3::Foward()
{
	return Vector3(0.0f, 0.0f, 1.0f);
}

inline bool Vector3::FindLineDefinedByTwoPlanes(const Vector3& planeP0, const Vector3& planeN0, const Vector3& planeP1, const Vector3& planeN1, Vector3& lineP0, Vector3& lineDir)
{
	lineDir = Vector3::Cross(planeN0, planeN1);

	auto mag = lineDir.Magnitude();

	if (mag > 0.0f)
	{
		lineDir /= mag;

		auto u = Vector3::Cross(planeN0, lineDir);

		if (LinePlaneIntersection(planeP1, planeN1, planeP0, planeP0 + u, lineP0))
			return true;
	}

	return false;
}

inline Vector3 Vector3::OrthogonalDirection(const Vector3& v)
{
	return Vector3::ProjectOnPlane(v, Vector3(-v.y, -v.z, v.x)).Normalized();
}