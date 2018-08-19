#pragma once
#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix.h"

class Transform
{
public:
	Vector3 GetPosition() const
	{
		return m_Position;
	}

	Quaternion GetRotation() const
	{
		return m_Rotation;
	}

	void SetPosition(const Vector3& pos)
	{
		m_Position = pos;
		m_Dirty = true;
	}

	void SetRotation(const Quaternion& rot)
	{
		m_Rotation = rot;
		m_Dirty = true;
	}

	void SetEqualTo(const Transform& other)
	{
		m_Position = other.GetPosition();
		m_Rotation = other.GetRotation();
		m_Dirty = true;
	}

	Vector3 ToLocalPosition(const Vector3& worldPos)
	{
		ReCalculateIfDirty();
		return m_WorldToLocal * worldPos;
	}

	Vector3 ToLocalDirection(const Vector3& worldDir) const
	{
		return m_Rotation.Conj().Rotate(worldDir);
	}

	Vector3 ToWorldPosition(const Vector3& localPos)
	{
		ReCalculateIfDirty();
		return m_LocalToWorld * localPos;
	}

	Vector3 ToWorldDirection(const Vector3& localDir)
	{
		return m_Rotation.Rotate(localDir);
	}

	Matrix4 GetLocalToWorldMatrix()
	{
		ReCalculateIfDirty();
		return m_LocalToWorld;
	}

	Matrix4 GetWorldToLocalMatrix()
	{
		ReCalculateIfDirty();
		return m_WorldToLocal;
	}

	Vector3 GetForward()
	{
		ReCalculateIfDirty();
		return FromColumn(m_LocalToWorld.M[2]);
	}

	Vector3 GetUp()
	{
		ReCalculateIfDirty();
		return FromColumn(m_LocalToWorld.M[1]);
	}

	Vector3 GetRight()
	{
		ReCalculateIfDirty();
		return FromColumn(m_LocalToWorld.M[0]);
	}

private:
	static inline Vector3 FromColumn(const float* col)
	{
		return Vector3(col[0], col[1], col[2]);
	}

	void ReCalculateIfDirty()
	{
		if (m_Dirty)
		{
			m_LocalToWorld = Matrix4::FromTranslation(m_Position) * Matrix4::FromRotation(m_Rotation);
			m_WorldToLocal = Matrix4::FromRotation(m_Rotation.Conj()) * Matrix4::FromTranslation(-m_Position);
			m_Dirty = false;
		}
	}

	Vector3 m_Position;
	Quaternion m_Rotation;

	Matrix4 m_LocalToWorld;
	Matrix4 m_WorldToLocal;

	bool m_Dirty;
};
