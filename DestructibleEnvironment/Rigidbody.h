#pragma once
#include "Vector3.h"
#include "Matrix.h"
#include "PhysicsObject.h"
#include "CollisionData.h"
#include "SplitInfo.h"

class Rigidbody : public PhysicsObject
{
public:
	void AddExternalImpulse(const Impulse& impulse)
	{
		m_ExternalImpulses.emplace_back(impulse);
	}

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

	void CopyMotionProperties(const Rigidbody& toCopy)
	{
		m_Drag = toCopy.m_Drag;
		m_AngularDrag = toCopy.m_AngularDrag;
		SetMass(toCopy.GetMass());
	}

	void CalculateMotionProperties();

	void AddForce(const Vector3& forceWorld)
	{
		m_ExternalForceWorld += forceWorld;
	}

	void AddMoment(const Vector3& momentWorld)
	{
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

	bool IsSplit() const
	{
		return m_IsSplit;
	}

	void ClearSplit()
	{
		m_IsSplit = false;
	}

	void ApplyImpulse(const Impulse& impulse) override;

private:
	void UpdateTransform();
	void CalculateInertia();

	Vector3 m_VelocityWorld;
	Vector3 m_AngularVelocityWorld;

	Vector3 m_ExternalForceWorld;
	Vector3 m_ExternalMomentsWorld;
	std::vector<Impulse> m_ExternalImpulses;

	float m_Drag;
	float m_AngularDrag;

	bool m_IsSplit = false;
};
