#pragma once

#include "Vector.h"
#include "Quaternion.h"
#include "Matrix.h"

class Transform
{
public:

	Vector3 GetPosition() const;
	Quaternion GetRotation() const;

	void SetPosition(const Vector3& pos);
	void SetRotation(const Quaternion rot);

	void SetEqualTo(const Transform& other);

	Vector3 ToLocalPosition(const Vector3& worldPos);
	Vector3 ToLocalDirection(const Vector3& worldDir);

	Vector3 ToWorldPosition(const Vector3& localPos);
	Vector3 ToWorldDirection(const Vector3& localDir);

	Matrix4 GetLocalToWorldMatrix() const;
};
