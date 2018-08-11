#pragma once

class Vector3
{

public:

	Vector3()
	{
	}

	Vector3(float xComp, float yComp, float zComp)
	{

	}

	float x;
	float y;
	float z;

	Vector3& operator-()
	{
		return *this;
	}

	void operator+=(const Vector3& rhs)
	{

	}

	void operator-=(const Vector3& rhs)
	{

	}

	// static

	static inline float Dot(const Vector3& v1, const Vector3& v2)
	{

	}

	static inline Vector3 Cross(const Vector3& v1, const Vector3& v2)
	{

	}

	static inline Vector3 ProjectOnPlane(const Vector3& planeNormal, const Vector3& vector)
	{

	}

	static inline Vector3 LinePlaneIntersection(const Vector3& planeP0, const Vector3& planeNormal, const Vector3& lineP0, const Vector3& lineP1)
	{

	}

	static inline Vector3 Zero()
	{

	}

	static inline Vector3 Normalize(const Vector3& v)
	{

	}
};

inline Vector3 operator+(const Vector3& lhs, const Vector3& rhs)
{

}

inline Vector3 operator-(const Vector3& lhs, const Vector3& rhs)
{

}

inline Vector3 operator/(const Vector3& lhs, float rhs)
{

}

inline Vector3 operator*(float lhs, const Vector3& rhs)
{

}