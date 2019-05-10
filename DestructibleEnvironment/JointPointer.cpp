#include "pch.h"
#include "JointPointer.h"
#include "PhysicsObject.h"

void JointPointer::Destroy()
{
	auto& j = GetValue();
	j.GetAnchorObj().RemoveJoint(*this);
	j.GetOtherObj().RemoveJoint(*this);
	m_Joints->Remove(m_Pointer);
}