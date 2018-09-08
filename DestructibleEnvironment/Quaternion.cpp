#include "pch.h"
#include "Quaternion.h"

Vector3 Quaternion::RotateV(const Vector3& v) const
{
	Vector3 u(x, y, z);

	auto s = r;

	auto v1 = (2.0f * Vector3::Dot(u, v) * u
		+ (s*s - Vector3::Dot(u, u)) * v
		+ 2.0f * s * Vector3::Cross(u, v));

	auto& thisRef = *this;
	auto p = Quaternion(0.0f, v.x, v.y, v.z);
	auto v2q = thisRef * (p * Conj());
	auto v2 = Vector3(v2q.x, v2q.y, v2q.z);

	return v1;
}