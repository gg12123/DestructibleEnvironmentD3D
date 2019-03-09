#pragma once
#include "CompoundShape.h"
#include "CollisionData.h"

class PhysicsObject : public CompoundShape
{
public:
	PhysicsObject()
	{
	}

	virtual void ApplyImpulse(const Impulse& impulse)
	{
	}

	float GetMass() const
	{
		return m_InvMass > 0.0f ? 1.0f / m_InvMass : MathU::Infinity;
	}

	float GetInvMass() const
	{
		return m_InvMass;
	}

	const Matrix3& GetInertiaInverseLocal() const
	{
		return m_InertiaInverse;
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
		m_InvMass = 1.0f / mass;
	}

	void SetInvMass(float invMass)
	{
		m_InvMass = invMass;
	}

	PhysicsObject* ToPhysicsObject() override
	{
		return this;
	}

protected:
	void SetInertia(const Matrix3& inertia)
	{
		m_InertiaInverse = Matrix3::Inverse(inertia);
	}

	void SetInvInertia(const Matrix3& invInertia)
	{
		m_InertiaInverse = invInertia;
	}

private:
	float m_InvMass;
	Matrix3 m_InertiaInverse;
};
