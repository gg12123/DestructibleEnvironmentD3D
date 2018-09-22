#pragma once
#include "Vector3.h"
#include "Matrix.h"
#include "PhysicsObject.h"
#include "CollisionData.h"
#include "SplitInfo.h"
#include "LastCheckedAgainst.h"

class Rigidbody : public PhysicsObject, public LastCheckedAgainst<Rigidbody*>
{
public:
	void AddImpulse(Impulse& impulse) override
	{
		m_Impulses.emplace_back(&impulse);
	}

	void AddToRequiredToSeperate(const Vector3& toSep) override
	{
		m_ToSeperate += toSep;
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

	void Update(std::vector<SplitInfo>& splits);

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
		m_AddedForceWorld += forceWorld;
	}

	void AddMoment(const Vector3& momentWorld)
	{
		m_AddedMomentsWorld += momentWorld;
	}

	void SetDrag(float drag)
	{
		m_Drag = drag;
	}

	void SetAngularDrag(float angDrag)
	{
		m_AngularDrag = angDrag;
	}

private:
	void UpdateTransform();
	void ApplyImpulses(std::vector<SplitInfo>& splits);
	void CalculateForces();
	void Integrate();
	void ApplyNormalForces();
	void ApplyImpulse(const Impulse& impulse);
	void CalculateInertia();

	std::vector<Impulse*> m_Impulses;

	Vector3 m_VelocityWorld;
	Vector3 m_AngularVelocityWorld;

	Vector3 m_AddedForceWorld;
	Vector3 m_AddedMomentsWorld;

	Vector3 m_ToSeperate;

	float m_Drag;
	float m_AngularDrag;
};
