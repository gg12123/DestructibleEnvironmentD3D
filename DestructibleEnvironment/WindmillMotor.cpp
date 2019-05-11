#include "pch.h"
#include "WindmillMotor.h"
#include "Rigidbody.h"
#include "World.h"

void WindmillMotor::Awake()
{
	GetWorld().RegisterEntityForUpdate(*this);
}

const Joint* WindmillMotor::FindAxisAndJoint()
{
	auto& rb = m_MillBase->GetPhysicsBody();

	for (auto& j : rb.GetJoints())
	{
		auto& joint = j.GetValue();

		if (!joint.GetRotConstraint1().IsConstrained())
		{
			m_Axis = joint.GetRotConstraint1().GetV();
			return &joint;
		}

		if (!joint.GetRotConstraint2().IsConstrained())
		{
			m_Axis = joint.GetRotConstraint2().GetV();
			return &joint;
		}

		if (!joint.GetRotConstraint3().IsConstrained())
		{
			m_Axis = joint.GetRotConstraint3().GetV();
			return &joint;
		}
	}

	return nullptr;
}

void WindmillMotor::Update()
{
	auto joint = FindAxisAndJoint();
	if (joint)
	{
		auto& rb = m_MillBase->GetPhysicsBody();
		auto& mill = joint->GetOther(rb);

		static constexpr auto demandVel = 1.0f;
		static constexpr auto gain = 5.0f;

		auto currVel = Vector3::Dot(mill.GetAngularVelocity(), m_Axis);

		// Should implement an add moment on physics body but
		// for now just do a hack dynamic cast
		auto millRb = dynamic_cast<Rigidbody*>(&mill);
		millRb->AddMoment(gain * (demandVel - currVel) * m_Axis);
	}
}