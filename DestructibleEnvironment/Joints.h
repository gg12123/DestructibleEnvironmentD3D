#pragma once
#include "Joint.h"
#include "JointPointer.h"

class Joints
{
public:
	JointPointer AddJoint(const Joint& j)
	{
		auto p = m_Joints.Add(j);
		return JointPointer(*p, m_Joints);
	}

	auto& GetJoints()
	{
		return m_Joints;
	}

private:
	SimdUnOrderedVector<Joint> m_Joints;
};
