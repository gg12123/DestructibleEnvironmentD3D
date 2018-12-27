#pragma once
#include "ShapeProxy.h"
#include "Vector3.h"
#include "CollisionData.h"

class Rigidbody;

class DynamicBodyProxy : public ShapeProxy
{
public:
	DynamicBodyProxy()
	{
	}

	DynamicBodyProxy(Rigidbody& body);

	float GetMass() const
	{
		return m_Mass;
	}

	void SetMass(float mass)
	{
		m_Mass = mass;
	}

	float GetDrag() const
	{
		return m_Drag;
	}

	void SetDrag(float drag)
	{
		m_Drag = drag;
	}

	float GetAngularDrag() const
	{
		return m_AngularDrag;
	}

	void SetAngularDrag(float angDrag)
	{
		m_AngularDrag = angDrag;
	}

	void AddForce(const Vector3& force);
	void AddTorque(const Vector3& torque);
	void AddImpulse(const Impulse& imp);

protected:
	Shape & RegisterWithPhysics() override;

	void SetRigidBody(Rigidbody& body)
	{
		m_Body = &body;
	}

private:
	float m_Mass;
	float m_Drag;
	float m_AngularDrag;

	Rigidbody* m_Body;
};
