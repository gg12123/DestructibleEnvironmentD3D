#pragma once
#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix.h"

class Transform
{
private:
	void ReCalculate()
	{
		m_LocalToWorld = Matrix4::FromTranslation(m_Position) * Matrix4::FromRotation(m_Rotation);
		m_WorldToLocal = Matrix4::FromRotation(m_Rotation.Conj()) * Matrix4::FromTranslation(-m_Position);
	}

public:
	const Vector3& GetPosition() const
	{
		return m_Position;
	}

	const Quaternion& GetRotation() const
	{
		return m_Rotation;
	}

	void SetPosition(const Vector3& pos)
	{
		m_Position = pos;
		ReCalculate();
	}

	void SetRotation(const Quaternion& rot)
	{
		m_Rotation = rot;
		m_Rotation.Normalize();
		ReCalculate();
	}

	void SetPositionAndRotation(const Vector3& pos, const Quaternion& rot)
	{
		m_Position = pos;
		m_Rotation = rot;
		m_Rotation.Normalize();
		ReCalculate();
	}

	void SetEqualTo(const Transform& other)
	{
		SetPositionAndRotation(other.GetPosition(), other.GetRotation());
	}

	Vector3 ToLocalPosition(const Vector3& worldPos) const
	{
		return m_WorldToLocal * worldPos;
	}

	Vector3 ToLocalDirection(const Vector3& worldDir) const
	{
		return GetWorldToLocalRotation().RotateV(worldDir);
	}

	Vector3 ToWorldPosition(const Vector3& localPos) const
	{
		return m_LocalToWorld * localPos;
	}

	Vector3 ToWorldDirection(const Vector3& localDir) const
	{
		return GetLocalToWorldRotation().RotateV(localDir);
	}

	Matrix3 ApplySimilarityTransform(const Matrix3& localMatrix) const
	{
		auto rotMat = m_LocalToWorld.ToMatrix3();
		return rotMat * localMatrix * rotMat.Transposed();
	}

	Matrix4 GetLocalToWorldMatrix() const
	{
		return m_LocalToWorld;
	}

	Matrix4 GetWorldToLocalMatrix() const
	{
		return m_WorldToLocal;
	}

	const Quaternion& GetLocalToWorldRotation() const
	{
		return m_Rotation;
	}

	Quaternion GetWorldToLocalRotation() const
	{
		return m_Rotation.Conj();
	}

	Vector3 GetForward() const
	{
		return FromColumn(m_LocalToWorld.Cols[2].Floats);
	}

	Vector3 GetUp() const
	{
		return FromColumn(m_LocalToWorld.Cols[1].Floats);
	}

	Vector3 GetRight() const
	{
		return FromColumn(m_LocalToWorld.Cols[0].Floats);
	}

private:
	static inline Vector3 FromColumn(const float* col)
	{
		return Vector3(col[0], col[1], col[2]);
	}

	Vector3 m_Position;
	Quaternion m_Rotation;

	Matrix4 m_LocalToWorld;
	Matrix4 m_WorldToLocal;
};
