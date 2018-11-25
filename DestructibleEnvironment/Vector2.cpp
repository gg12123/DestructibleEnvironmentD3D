#include "pch.h"
#include <assert.h>
#include "Vector2.h"

Vector2 Vector2::Normalized() const
{
	auto mag = Magnitude();
	assert(mag > 0.0f);
	return *this / mag;
}