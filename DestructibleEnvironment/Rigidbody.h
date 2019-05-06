#pragma once
#include "Vector3.h"
#include "Matrix.h"
#include "PhysicsObject.h"
#include "CollisionData.h"
#include "SplitInfo.h"

class Island;

class Rigidbody : public PhysicsObject
{
public:
	Rigidbody()
	{
		SetAwake(true);
		SetStatic(false);
	}

	void ClearIsland()
	{
		m_Island = nullptr;
	}

	void SetIsland(const Island& island)
	{
		m_Island = &island;
	}

	const Island* GetIsland() const
	{
		return m_Island;
	}

	bool IsStill() const
	{
		// Check if the angular and linear veloctiy has been below some tol for
		// a few ticks.
		return false;
	}

	void GoToSleep()
	{
		SetAwake(false);
	}

	void WakeUp()
	{
		SetAwake(true);
	}

	void InitMassProperties(const Transform& refTran);

	const Vector3& GetVelocityWorld() const
	{
		return m_VelocityWorld;
	}

	const Vector3& GetAngularVelocityWorld() const
	{
		return m_AngularVelocityWorld;
	}

	Vector3 GetVelocityLocal()
	{
		return GetTransform().ToLocalDirection(m_VelocityWorld);
	}

	Vector3 GetAngularVelocityLocal()
	{
		return GetTransform().ToLocalDirection(m_AngularVelocityWorld);
	}

	Vector3 WorldVelocityAt(const Vector3& worldPoint) override
	{
		return m_VelocityWorld + Vector3::Cross(m_AngularVelocityWorld, worldPoint - GetTransform().GetPosition());
	}

	void UpdatePosition();

	void CopyVelocity(const Rigidbody& toCopy)
	{
		m_VelocityWorld = toCopy.m_VelocityWorld;
		m_AngularVelocityWorld = toCopy.m_AngularVelocityWorld;
	}

	void CopyDrag(const Rigidbody& toCopy)
	{
		m_Drag = toCopy.m_Drag;
		m_AngularDrag = toCopy.m_AngularDrag;
	}

	void AddForce(const Vector3& forceWorld)
	{
		SetAwake(true);
		m_ExternalForceWorld += forceWorld;
	}

	void AddMoment(const Vector3& momentWorld)
	{
		SetAwake(true);
		m_ExternalMomentsWorld += momentWorld;
	}

	void SetDrag(float drag)
	{
		m_Drag = drag;
	}

	void SetAngularDrag(float angDrag)
	{
		m_AngularDrag = angDrag;
	}

	void ApplyExternalForcesAndImpulses();

	void ApplyImpulse(const Impulse& impulse) override;

private:
	void UpdateTransform();

	Vector3 m_VelocityWorld;
	Vector3 m_AngularVelocityWorld;

	Vector3 m_ExternalForceWorld;
	Vector3 m_ExternalMomentsWorld;

	float m_Drag;
	float m_AngularDrag;

	const Island* m_Island;
};
