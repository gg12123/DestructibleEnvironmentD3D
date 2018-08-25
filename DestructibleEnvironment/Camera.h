#pragma once
#include "Entity.h"
#include "Matrix.h"
#include "Transform.h"

class Camera : public Entity
{
public:
	Matrix4 CalcuateVPMatrix()
	{
		return Matrix4::Perspective(m_Fov, m_Aspect, m_NearClip, m_FarClip) *
			GetTransform().GetWorldToLocalMatrix();
	}

	Vector3 GetViewDirection()
	{
		return GetTransform().GetForward();
	}

	void SetNearClip(float val)
	{
		m_NearClip = val;
	}

	void SetFarClip(float val)
	{
		m_FarClip = val;
	}

	void SetFov(float val)
	{
		m_Fov = val;
	}

protected:

	void Awake() override;

private:
	float m_Fov;
	float m_Aspect = 1.0f; // this needs to come from the window config - probs through the renderer
	float m_NearClip;
	float m_FarClip;
};
