#pragma once
#include "ShapeProxy.h"
#include "Vector3.h"

class DynamicBodyProxy : public ShapeProxy
{
public:
	DynamicBodyProxy()
	{
	}

	DynamicBodyProxy(Shape& body);

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

protected:
	Shape & RegisterWithPhysics() override;

private:
	float m_Mass;
	float m_Drag;
	float m_AngularDrag;
};
