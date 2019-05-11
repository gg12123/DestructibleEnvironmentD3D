#pragma once
#include "Vector3.h"
#include "Matrix.h"
#include "PhysicsObject.h"
#include "CollisionData.h"
#include "SplitInfo.h"

class Island;

class RigidbodyStillnessMonitor
{
public:
	void Tick(const Vector3& linVel, const Vector3& angVel)
	{
		static constexpr auto linearTol = 0.0001f;
		static constexpr auto angTol = 0.0001f;

		static constexpr auto linearTolSqr = linearTol * linearTol;
		static constexpr auto angTolSqr = angTol * angTol;

		if (linVel.MagnitudeSqr() <= linearTolSqr && angVel.MagnitudeSqr() <= angTolSqr)
		{
			m_NumTicksBelowTol++;
		}
		else
		{
			m_NumTicksBelowTol = 0;
		}
	}

	bool IsStill() const
	{
		static constexpr auto numTicksRequiredForStill = 2;
		return m_NumTicksBelowTol >= numTicksRequiredForStill;
	}

	void Reset()
	{
		m_NumTicksBelowTol = 0;
	}

private:
	int m_NumTicksBelowTol = 0;
};

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

	bool IsStill() const;

	void GoToSleep()
	{
		SetAwake(false);
	}

	void WakeUp()
	{
		SetAwake(true);
		m_StillnessMonitor.Reset();
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

	Vector3 WorldVelocityAt(const Vector3& worldPoint) const override
	{
		return m_VelocityWorld + Vector3::Cross(m_AngularVelocityWorld, worldPoint - GetTransform().GetPosition());
	}

	virtual Vector3 GetAngularVelocity() const override
	{
		return m_AngularVelocityWorld;
	}

	void UpdatePosition();

	void CopyVelocity(const Rigidbody& toCopy)
	{
		m_VelocityWorld = toCopy.m_VelocityWorld;
		m_AngularVelocityWorld = toCopy.m_AngularVelocityWorld;
	}

	void CopyDamping(const Rigidbody& toCopy)
	{
		m_LinearDamping = toCopy.m_LinearDamping;
		m_AngularDamping = toCopy.m_AngularDamping;
	}

	void AddForce(const Vector3& forceWorld)
	{
		WakeUp();
		m_ExternalForceWorld += forceWorld;
	}

	void AddMoment(const Vector3& momentWorld)
	{
		WakeUp();
		m_ExternalMomentsWorld += momentWorld;
	}

	void SetLinearDamping(float val)
	{
		m_LinearDamping = MathU::Clamp(val, 0.0f, 0.99f);
	}

	void SetAngularDamping(float val)
	{
		m_AngularDamping = MathU::Clamp(val, 0.0f, 0.99f);
	}

	void ApplyExternalForcesAndImpulses();

	// For impulses applied by the solver
	void ApplyImpulse(const Impulse& impulse) override;
	void ApplyAngularImpulse(const Vector3& impulse) override;

	// For impulses applied by the game world
	void ApplyExternalImpulse(const Impulse& impulse);

private:
	void UpdateTransform();

	Vector3 m_VelocityWorld;
	Vector3 m_AngularVelocityWorld;

	Vector3 m_ExternalForceWorld;
	Vector3 m_ExternalMomentsWorld;

	float m_LinearDamping;
	float m_AngularDamping;

	const Island* m_Island;
	RigidbodyStillnessMonitor m_StillnessMonitor;
};
