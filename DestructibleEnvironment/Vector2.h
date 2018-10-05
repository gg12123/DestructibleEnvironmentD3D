#pragma once

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

	Vector2 Normalized()
	{

	}

	float Magnitude() const
	{

	}

	// static

	static inline Vector2 Up();
	static inline Vector2 Right();
	static inline float Dot(const Vector2& v1, const Vector2& v2);
	static inline bool LinesIntersect(const Vector2& aP0, const Vector2& aP1, const Vector2& bP0, const Vector2& bP1, Vector2& intPoint);
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

}

inline Vector2 operator/(const Vector2& lhs, float rhs)
{

}