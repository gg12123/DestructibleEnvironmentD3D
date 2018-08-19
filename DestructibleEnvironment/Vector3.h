#pragma once
#include <math.h>

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

	Vector3 operator-()
	{
		return Vector3(-x, -y, -x);
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

	float Magnitude()
	{
		return sqrt(x * x + y * y + z * z);
	}

	// static

	static inline float Dot(const Vector3& v1, const Vector3& v2);
	static inline Vector3 Cross(const Vector3& v1, const Vector3& v2);
	static inline Vector3 ProjectOnPlane(const Vector3& planeNormal, const Vector3& vector);
	static inline Vector3 LinePlaneIntersection(const Vector3& planeP0, const Vector3& planeNormal, const Vector3& lineP0, const Vector3& lineP1);
	static inline Vector3 Zero();
	static inline Vector3 Normalize(const Vector3& v);
	static inline Vector3 Right();
	static inline Vector3 Up();
	static inline Vector3 Foward();
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

inline Vector3 Vector3::LinePlaneIntersection(const Vector3& planeP0, const Vector3& planeNormal, const Vector3& lineP0, const Vector3& lineP1)
{
	auto l = Normalize(lineP1 - lineP0);

	auto num = Vector3::Dot(planeP0 - lineP0, planeNormal);
	auto denom = Vector3::Dot(l, planeNormal);

	assert(denom != 0.0f);

	auto u = num / denom;

	return lineP0 + u * l;
}

inline Vector3 Vector3::Zero()
{
	return Vector3(0.0f, 0.0f, 0.0f);
}

inline Vector3 Vector3::Normalize(const Vector3& v)
{
	auto res = v;
	res /= res.Magnitude();
	return res;
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