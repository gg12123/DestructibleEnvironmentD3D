#pragma once
#include "Vector3.h"
#include "Matrix.h"
#include "PhysicsObject.h"
#include "CollisionData.h"
#include "SplitInfo.h"

class Rigidbody : public PhysicsObject
{
public:
	void AddImpulse(Impulse& impulse) override
	{
		m_Impulses.emplace_back(&impulse);
	}

	const Vector3& GetVelocityWorld() const
	{
		return m_VelocityWorld;
	}

	const Vector3& GetAngularVelocityLocal() const
	{
		return m_AngularVelocityLocal;
	}

	Vector3 GetVelocityLocal()
	{
		return GetTransform().ToLocalDirection(m_VelocityWorld);
	}

	Vector3 GetAngularVelocityWorld()
	{
		return GetTransform().ToWorldDirection(m_AngularVelocityLocal);
	}

	Vector3 WorldVelocityAt(const Vector3& worldPoint) override
	{
		return m_VelocityWorld + Vector3::Cross(GetAngularVelocityWorld(), worldPoint - GetTransform().GetPosition());
	}

	void Update(std::vector<SplitInfo>& splits);

	void CopyVelocity(const Rigidbody& toCopy)
	{
		m_VelocityWorld = toCopy.m_VelocityWorld;
		m_AngularVelocityLocal = toCopy.m_AngularVelocityLocal;
	}

private:
	void UpdateTransform();
	void ApplyImpulses(std::vector<SplitInfo>& splits);
	void CalculateForces(Vector3& forcesWorld, Vector3& momentsLocal);
	void Integrate(const Vector3& forcesWorld, const Vector3& momentsLocal);
	void ApplyNormalForces();
	void ApplyImpulse(const Impulse& impulse);

	std::vector<Impulse*> m_Impulses;

	Vector3 m_VelocityWorld;
	Vector3 m_AngularVelocityLocal;

	float m_Drag;
	float m_AngularDrag;
};
