#pragma once
#include "Shape.h"
#include "CollisionData.h"

class PhysicsObject : public Shape
{
public:
	virtual void AddImpulse(Impulse& impulse)
	{
	}

	float GetMass() const
	{
		return m_Mass;
	}

	const Matrix3& GetInertia() const
	{
		return m_Inertia;
	}

	const Matrix3& GetInertiaInverse() const
	{
		return m_InertiaInverse;
	}

	virtual Vector3 WorldVelocityAt(const Vector3& worldPoint)
	{
		return Vector3::Zero();
	}

private:
	float m_Mass;
	Matrix3 m_Inertia;
	Matrix3 m_InertiaInverse;
};