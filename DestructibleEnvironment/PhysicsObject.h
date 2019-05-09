#pragma once
#include "CompoundShape.h"
#include "CollisionData.h"

class PhysicsObject : public CompoundShape
{
public:
	virtual void ApplyImpulse(const Impulse& impulse)
	{
	}

	virtual void ApplyAngularImpulse(const Vector3& impulse)
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
		return m_InertiaInverseLocal;
	}

	const Matrix3& GetInertiaInverseWorld() const
	{
		return m_InertiaInverseWorld;
	}

	virtual Vector3 WorldVelocityAt(const Vector3& worldPoint) const
	{
		return Vector3::Zero();
	}

	virtual Vector3 GetAngularVelocity() const
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
		m_InertiaInverseLocal = Matrix3::Inverse(inertia);
		UpdateWorldInertiaInverse();
	}

	void SetInvInertia(const Matrix3& invInertia)
	{
		m_InertiaInverseLocal = invInertia;
		UpdateWorldInertiaInverse();
	}

	void UpdateWorldInertiaInverse()
	{
		m_InertiaInverseWorld = GetTransform().ApplySimilarityTransform(m_InertiaInverseLocal);
	}

private:
	float m_InvMass;
	Matrix3 m_InertiaInverseLocal;
	Matrix3 m_InertiaInverseWorld;
};
