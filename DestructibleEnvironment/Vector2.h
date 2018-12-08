#pragma once
#include <math.h>

class Vector2
{
public:
	float x;
	float y;

	Vector2()
	{
		x = y = 0.0f;
	}

	Vector2(float xVal, float yVal)
	{
		x = xVal;
		y = yVal;
	}

	void operator += (const Vector2& rhs)
	{
		x += rhs.x;
		y += rhs.y;
	}

	Vector2 operator-() const
	{
		return Vector2(-x, -y);
	}

	Vector2 Normalized() const;

	float Magnitude() const
	{
		return sqrt(x * x + y * y);
	}

	// static

	static inline Vector2 Zero();
	static inline Vector2 Up();
	static inline Vector2 Right();
	static inline float Dot(const Vector2& v1, const Vector2& v2);
	static inline bool InfinateLinesIntersect(const Vector2& aP0, const Vector2& aP1, const Vector2& bP0, const Vector2& bP1, float& aU, float& bU);
	static inline bool LinesIntersect(const Vector2& aP0, const Vector2& aP1, const Vector2& bP0, const Vector2& bP1, Vector2& intPoint);
	static inline bool RayIntersectsLine(const Vector2& origin, const Vector2& dir, const Vector2& P0, const Vector2& P1, Vector2& intPoint);
	static inline bool InfinateLineIntersectsLine(const Vector2& infP0, const Vector2& infP1, const Vector2& P0, const Vector2& P1, Vector2& intPoint);
	static inline float Cross2D(const Vector2& u, const Vector2& v);
};

inline Vector2 operator+(const Vector2& lhs, const Vector2& rhs)
{
	return Vector2(lhs.x + rhs.x, lhs.y + rhs.y);
}

inline Vector2 operator-(const Vector2& lhs, const Vector2& rhs)
{
	return Vector2(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline Vector2 operator*(float lhs, const Vector2& rhs)
{
	return Vector2(lhs * rhs.x, lhs * rhs.y);
}

inline Vector2 operator/(const Vector2& lhs, float rhs)
{
	return Vector2(lhs.x / rhs, lhs.y / rhs);
}

inline float Vector2::Cross2D(const Vector2& u, const Vector2& v)
{
	return (u.x * v.y - u.y * v.x);
}

inline Vector2 Vector2::Zero()
{
	return Vector2(0.0f, 0.0f);
}

inline Vector2 Vector2::Up()
{
	return Vector2(0.0f, 1.0f);
}

inline Vector2 Vector2::Right()
{
	return Vector2(1.0f, 0.0f);
}

inline float Vector2::Dot(const Vector2& v1, const Vector2& v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

inline bool Vector2::InfinateLinesIntersect(const Vector2& aP0, const Vector2& aP1, const Vector2& bP0, const Vector2& bP1, float& aU, float& bU)
{
	auto x1 = aP0.x;
	auto y1 = aP0.y;

	auto x2 = aP1.x;
	auto y2 = aP1.y;

	auto x3 = bP0.x;
	auto y3 = bP0.y;

	auto x4 = bP1.x;
	auto y4 = bP1.y;

	auto denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

	if (denom != 0.0f)
	{
		aU = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
		bU = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
		return true;
	}
	return false;
}

inline bool Vector2::LinesIntersect(const Vector2& aP0, const Vector2& aP1, const Vector2& bP0, const Vector2& bP1, Vector2& intPoint)
{
	float aU, bU;
	if (InfinateLinesIntersect(aP0, aP1, bP0, bP1, aU, bU))
	{
		intPoint = aP0 + aU * (aP1 - aP0);
		return aU >= 0.0f && aU <= 1.0f && bU >= 0.0f && bU <= 1.0f;
	}
	return false;
}

inline bool Vector2::RayIntersectsLine(const Vector2& origin, const Vector2& dir, const Vector2& P0, const Vector2& P1, Vector2& intPoint)
{
	auto& rayP0 = origin;
	auto rayP1 = origin + dir;

	float aU, bU;
	if (InfinateLinesIntersect(rayP0, rayP1, P0, P1, aU, bU))
	{
		intPoint = rayP0 + aU * (rayP1 - rayP0);
		return aU >= 0.0f && bU >= 0.0f && bU <= 1.0f;
	}
	return false;
}

inline bool Vector2::InfinateLineIntersectsLine(const Vector2& infP0, const Vector2& infP1, const Vector2& P0, const Vector2& P1, Vector2& intPoint)
{
	float aU, bU;
	if (InfinateLinesIntersect(infP0, infP1, P0, P1, aU, bU))
	{
		intPoint = infP0 + aU * (infP1 - infP0);
		return bU >= 0.0f && bU <= 1.0f;
	}
	return false;
}