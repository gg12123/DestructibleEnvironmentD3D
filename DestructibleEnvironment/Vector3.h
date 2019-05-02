#pragma once
#include <math.h>
#include <xmmintrin.h>
#include "MathU.h"
#include "Simd.h"

#ifndef USE_SIMD

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
	static inline Vector3 Normalize(const Vector3& v);
	static inline Vector3 Zero();
	static inline Vector3 Right();
	static inline Vector3 Up();
	static inline Vector3 Foward();
	static  Vector3 OrthogonalDirection(const Vector3& v);
	static inline void CalculateBaryCords(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& p, float& a, float& b, float& c);
	static inline bool FindClosestPointsBetweenLines(const Vector3& l0A, const Vector3& l0B, const Vector3& l1A, const Vector3& l1B, Vector3& l0p, Vector3& l1p);
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

inline Vector3 Vector3::Normalize(const Vector3& v)
{
	auto res = v;
	auto mag = res.Magnitude();
	return (mag > 0.0f ? res / mag : Vector3::Zero());
}

inline Vector3 Vector3::Zero()
{
	return Vector3(0.0f, 0.0f, 0.0f);
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

// The barycentric coordinates of p in triangle ABC
inline void Vector3::CalculateBaryCords(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& p, float& a, float& b, float& c)
{
	auto v0 = B - A;
	auto v1 = C - A;
	auto v2 = p - A;

	auto d00 = Dot(v0, v0);
	auto d01 = Dot(v0, v1);
	auto d11 = Dot(v1, v1);
	auto d20 = Dot(v2, v0);
	auto d21 = Dot(v2, v1);

	auto denom = d00 * d11 - d01 * d01;
	b = (d11 * d20 - d01 * d21) / denom;
	c = (d00 * d21 - d01 * d20) / denom;
	a = 1 - b - c;
}

inline bool Vector3::FindClosestPointsBetweenLines(const Vector3& p1, const Vector3& q1, const Vector3& p2, const Vector3& q2, Vector3& c1, Vector3& c2)
{
	auto d1 = q1 - p1;
	auto d2 = q2 - p2;
	auto r = p1 - p2;

	auto a = Vector3::Dot(d1, d1);
	auto b = Vector3::Dot(d1, d2);
	auto c = Vector3::Dot(d1, r);
	auto e = Vector3::Dot(d2, d2);
	auto f = Vector3::Dot(d2, r);
	auto d = a * e - b * b;

	if (MathU::Abs(d) > MathU::Epsilon)
	{
		auto s = (b * f - c * e) / d;
		auto t = (a * f - b * c) / d;

		c1 = p1 + s * d1;
		c2 = p2 + t * d2;

		return true;
	}
	return false;
}

#else

#define _MY_SHUFFLE(a, b, c, d)  _MM_SHUFFLE(d, c, b, a)

static float Vec128HorizontalSum(__m128 v) // v = [ D C | B A ]
{
	__m128 shuf = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 3, 0, 1));  // [ C D | A B ]
	__m128 sums = _mm_add_ps(v, shuf);      // sums = [ D+C C+D | B+A A+B ]
	shuf = _mm_movehl_ps(shuf, sums);      //  [   C   D | D+C C+D ]  // let the compiler avoid a mov by reusing shuf
	sums = _mm_add_ss(sums, shuf);
	return _mm_cvtss_f32(sums);
}

class SimdVector3
{
private:

public:
	union
	{
		__m128 Vec128;
		float Floats[4];
	};

	SimdVector3()
	{
		Vec128 = _mm_set1_ps(0.0f);
	}

	SimdVector3(float x, float y, float z)
	{
		Floats[0] = x;
		Floats[1] = y;
		Floats[2] = z;
		Floats[3] = 0.0f;
	}

	SimdVector3(const __m128& vec128) : Vec128(vec128)
	{
	}

	auto X() const
	{
		return Floats[0];
	}

	auto Y() const
	{
		return Floats[1];
	}

	auto Z() const
	{
		return Floats[2];
	}

	SimdVector3 operator-() const
	{
		auto m = _mm_set1_ps(-1.0f);
		return SimdVector3(_mm_mul_ps(m, Vec128));
	}

	SimdVector3& operator*=(float rhs)
	{
		auto m = _mm_set1_ps(rhs);
		Vec128 = _mm_mul_ps(m, Vec128);
		return *this;
	}

	void operator+=(const SimdVector3& rhs)
	{
		Vec128 = _mm_add_ps(Vec128, rhs.Vec128);
	}

	void operator-=(const SimdVector3& rhs)
	{
		Vec128 = _mm_sub_ps(Vec128, rhs.Vec128);
	}

	void operator/=(float rhs)
	{
		*this *= (1.0f / rhs);
	}

	float Magnitude() const
	{
		return sqrt(MagnitudeSqr());
	}

	float MagnitudeSqr() const
	{
		return Dot(*this, *this);
	}

	SimdVector3 InDirectionOf(const SimdVector3& refDir) const
	{
		auto& thisRef = *this;
		return (SimdVector3::Dot(thisRef, refDir) < 0.0f) ? -thisRef : thisRef;
	}

	SimdVector3 Normalized() const
	{
		return SimdVector3::Normalize(*this);
	}

	void Normalize()
	{
		auto mag = Magnitude();

		if (mag > 0.0f)
			(*this) /= Magnitude();
	}

	static inline float Dot(const SimdVector3& lhs, const SimdVector3& rhs)
	{
		auto x = _mm_mul_ps(lhs.Vec128, rhs.Vec128);
		return Vec128HorizontalSum(x);
	}

	static inline SimdVector3 Cross(const SimdVector3& a, const SimdVector3& b)
	{
		auto aShuf = _mm_shuffle_ps(a.Vec128, a.Vec128, _MY_SHUFFLE(1, 2, 0, 3));
		auto bShuf = _mm_shuffle_ps(b.Vec128, b.Vec128, _MY_SHUFFLE(1, 2, 0, 3));
	
		aShuf = _mm_mul_ps(aShuf, b.Vec128);
		bShuf = _mm_mul_ps(bShuf, a.Vec128);

		bShuf = _mm_sub_ps(bShuf, aShuf);
	
		return SimdVector3(_mm_shuffle_ps(bShuf, bShuf, _MY_SHUFFLE(1, 2, 0, 3)));
	}

	static inline SimdVector3 ProjectOnPlane(const SimdVector3& planeNormal, const SimdVector3& vector);
	static inline bool LinePlaneIntersection(const SimdVector3& planeP0, const SimdVector3& planeNormal, const SimdVector3& lineP0, const SimdVector3& lineP1, SimdVector3& intPoint);
	static inline bool FindLineDefinedByTwoPlanes(const SimdVector3& planeP0, const SimdVector3& planeN0, const SimdVector3& planeP1, const SimdVector3& planeN1, SimdVector3& lineP0, SimdVector3& lineDir);
	static inline SimdVector3 Normalize(const SimdVector3& v);
	static inline SimdVector3 Zero();
	static inline SimdVector3 Right();
	static inline SimdVector3 Up();
	static inline SimdVector3 Foward();
	static  SimdVector3 OrthogonalDirection(const SimdVector3& v);
	static inline void CalculateBaryCords(const SimdVector3& A, const SimdVector3& B, const SimdVector3& C, const SimdVector3& p, float& a, float& b, float& c);
	static inline bool FindClosestPointsBetweenLines(const SimdVector3& l0A, const SimdVector3& l0B, const SimdVector3& l1A, const SimdVector3& l1B, SimdVector3& l0p, SimdVector3& l1p);
};

inline SimdVector3 operator+(const SimdVector3& lhs, const SimdVector3& rhs)
{
	return SimdVector3(_mm_add_ps(lhs.Vec128, rhs.Vec128));
}

inline SimdVector3 operator-(const SimdVector3& lhs, const SimdVector3& rhs)
{
	return SimdVector3(_mm_sub_ps(lhs.Vec128, rhs.Vec128));
}

inline SimdVector3 operator*(float lhs, const SimdVector3& rhs)
{
	return SimdVector3(_mm_mul_ps(_mm_set1_ps(lhs), rhs.Vec128));
}

inline SimdVector3 operator*(const SimdVector3& lhs, float rhs)
{
	return rhs * lhs;
}

inline SimdVector3 operator/(const SimdVector3& lhs, float rhs)
{
	return (1.0f / rhs) * lhs;
}

inline SimdVector3 SimdVector3::ProjectOnPlane(const SimdVector3& planeNormal, const SimdVector3& vector)
{
	return vector - (planeNormal * Dot(vector, planeNormal));
}

// assuming that the line intersect the plane
inline bool SimdVector3::LinePlaneIntersection(const SimdVector3& planeP0, const SimdVector3& planeNormal, const SimdVector3& lineP0, const SimdVector3& lineP1, SimdVector3& intPoint)
{
	auto l = Normalize(lineP1 - lineP0);

	auto num = SimdVector3::Dot(planeP0 - lineP0, planeNormal);
	auto denom = SimdVector3::Dot(l, planeNormal);

	if (denom == 0.0f)
		return false;

	auto u = num / denom;

	intPoint = lineP0 + u * l;
	return true;
}

inline SimdVector3 SimdVector3::Normalize(const SimdVector3& v)
{
	auto res = v;
	auto mag = res.Magnitude();
	return (mag > 0.0f ? res / mag : SimdVector3::Zero());
}

inline SimdVector3 SimdVector3::Zero()
{
	return SimdVector3(0.0f, 0.0f, 0.0f);
}

inline SimdVector3 SimdVector3::Right()
{
	return SimdVector3(1.0f, 0.0f, 0.0f);
}

inline SimdVector3 SimdVector3::Up()
{
	return SimdVector3(0.0f, 1.0f, 0.0f);
}

inline SimdVector3 SimdVector3::Foward()
{
	return SimdVector3(0.0f, 0.0f, 1.0f);
}

inline bool SimdVector3::FindLineDefinedByTwoPlanes(const SimdVector3& planeP0, const SimdVector3& planeN0, const SimdVector3& planeP1, const SimdVector3& planeN1, SimdVector3& lineP0, SimdVector3& lineDir)
{
	lineDir = SimdVector3::Cross(planeN0, planeN1);

	auto mag = lineDir.Magnitude();

	if (mag > 0.0f)
	{
		lineDir /= mag;

		auto u = SimdVector3::Cross(planeN0, lineDir);

		if (LinePlaneIntersection(planeP1, planeN1, planeP0, planeP0 + u, lineP0))
			return true;
	}
	return false;
}

inline SimdVector3 SimdVector3::OrthogonalDirection(const SimdVector3& v)
{
	return SimdVector3::ProjectOnPlane(v, SimdVector3(-v.Y(), -v.Z(), v.X())).Normalized();
}

// The barycentric coordinates of p in triangle ABC
inline void SimdVector3::CalculateBaryCords(const SimdVector3& A, const SimdVector3& B, const SimdVector3& C, const SimdVector3& p, float& a, float& b, float& c)
{
	auto v0 = B - A;
	auto v1 = C - A;
	auto v2 = p - A;

	auto d00 = Dot(v0, v0);
	auto d01 = Dot(v0, v1);
	auto d11 = Dot(v1, v1);
	auto d20 = Dot(v2, v0);
	auto d21 = Dot(v2, v1);

	auto denom = d00 * d11 - d01 * d01;
	b = (d11 * d20 - d01 * d21) / denom;
	c = (d00 * d21 - d01 * d20) / denom;
	a = 1 - b - c;
}

inline bool SimdVector3::FindClosestPointsBetweenLines(const SimdVector3& p1, const SimdVector3& q1, const SimdVector3& p2, const SimdVector3& q2, SimdVector3& c1, SimdVector3& c2)
{
	auto d1 = q1 - p1;
	auto d2 = q2 - p2;
	auto r = p1 - p2;

	auto a = SimdVector3::Dot(d1, d1);
	auto b = SimdVector3::Dot(d1, d2);
	auto c = SimdVector3::Dot(d1, r);
	auto e = SimdVector3::Dot(d2, d2);
	auto f = SimdVector3::Dot(d2, r);
	auto d = a * e - b * b;

	if (MathU::Abs(d) > MathU::Epsilon)
	{
		auto s = (b * f - c * e) / d;
		auto t = (a * f - b * c) / d;

		c1 = p1 + s * d1;
		c2 = p2 + t * d2;

		return true;
	}
	return false;
}

using Vector3 = SimdVector3;

#endif // !USE_SIMD