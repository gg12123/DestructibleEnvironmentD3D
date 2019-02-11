#pragma once
#include "Vector3.h"
#include "Matrix.h"
#include "PhysicsObject.h"
#include "CollisionData.h"
#include "SplitInfo.h"

class Rigidbody : public PhysicsObject
{
public:
	void AddImpulse(const Impulse& impulse) override
	{
		// Impulses added by collision response
		m_Impulses.emplace_back(impulse);
	}

	void AddAdditionalImpulse(const Impulse& impulse)
	{
		// Impulses added by the game thread
		m_AdditionalImpulses.emplace_back(impulse);
	}

	void AddContact(const ContactManifold& contact) override
	{
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

	void UpdatePosition(std::vector<SplitInfo>& splits);

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

	void ApplyExternalForces();

private:
	void TransferAdditionalImpulses()
	{
		m_Impulses.insert(m_Impulses.end(), m_AdditionalImpulses.begin(), m_AdditionalImpulses.end());
		m_AdditionalImpulses.clear();
	}

	void UpdateTransform();
	void RewindIfPenetrating();
	void SatisfyContactConstraints();
	void ApplyImpulses(std::vector<SplitInfo>& splits);
	void ApplyImpulse(const Impulse& impulse);
	void CalculateInertia();

	std::vector<Impulse> m_Impulses;
	std::vector<Impulse> m_AdditionalImpulses;
	std::vector<ContactManifold> m_Contacts;

	Vector3 m_VelocityWorld;
	Vector3 m_AngularVelocityWorld;

	Vector3 m_AddedForceWorld;
	Vector3 m_AddedMomentsWorld;

	Vector3 m_ToSeperate;

	float m_Drag;
	float m_AngularDrag;
};
