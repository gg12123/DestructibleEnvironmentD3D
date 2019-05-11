#pragma once
#include "Joint.h"
#include "JointPointer.h"
#include "PhysicsObject.h"

class Joints
{
public:
	JointPointer AddJoint(const Joint& j)
	{
		auto pp = m_Joints.Add(j);
		auto p = JointPointer(*pp, m_Joints);
		j.GetAnchorObj().AddJoint(p);
		j.GetOtherObj().AddJoint(p);
		return p;
	}

	auto& GetJoints()
	{
		return m_Joints;
	}

private:
	SimdUnOrderedVector<Joint> m_Joints;
};
