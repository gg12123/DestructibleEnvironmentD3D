#pragma once
#include "Entity.h"
#include "Matrix.h"
#include "Transform.h"
#include "RayCasting.h"
#include "Vector2.h"

class Camera : public Entity
{
public:
	Matrix4 CalcuateVPMatrix();

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

	Ray ScreenPointToRay(const Vector2& p);

protected:
	void Awake() override;

private:
	float m_Fov; // In radians
	float m_NearClip;
	float m_FarClip;
};
