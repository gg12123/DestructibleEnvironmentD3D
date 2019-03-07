#pragma once
#include "CompoundShape.h"
#include "CollisionData.h"

class PhysicsObject : public CompoundShape
{
public:
	PhysicsObject()
	{
		m_Inertia = Matrix3::Indentity();
		m_InertiaInverse = Matrix3::Indentity();
	}

	virtual void ApplyImpulse(const Impulse& impulse)
	{
	}

	float GetMass() const
	{
		return m_Mass;
	}

	const Matrix3& GetInertiaLocal() const
	{
		return m_Inertia;
	}

	const Matrix3& GetInertiaInverseLocal() const
	{
		return m_InertiaInverse;
	}

	Matrix3 GetInertiaWorld()
	{
		return GetTransform().ApplySimilarityTransform(m_Inertia);
	}

	Matrix3 GetInertiaInverseWorld()
	{
		return GetTransform().ApplySimilarityTransform(m_InertiaInverse);
	}

	virtual Vector3 WorldVelocityAt(const Vector3& worldPoint)
	{
		return Vector3::Zero();
	}

	void SetMass(float mass)
	{
		m_Mass = mass;
	}

	PhysicsObject* ToPhysicsObject() override
	{
		return this;
	}

protected:
	void SetInertia(const Matrix3& inertia)
	{
		m_Inertia = inertia;
		m_InertiaInverse = Matrix3::Inverse(inertia);
	}

private:
	float m_Mass;
	Matrix3 m_Inertia;
	Matrix3 m_InertiaInverse;
};
