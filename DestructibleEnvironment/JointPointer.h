#pragma once
#include "Joint.h"

class JointPointer
{
public:
	JointPointer() : m_Pointer(nullptr), m_Joints(nullptr)
	{
	}

	JointPointer(SimdUnOrderedVector<Joint>::Pointer& p, SimdUnOrderedVector<Joint>& joints) :
		m_Pointer(&p), m_Joints(&joints)
	{
	}

	const Joint& GetValue() const
	{
		return m_Joints->GetValue(m_Pointer);
	}

	void Destroy();

	bool operator == (const JointPointer& other)
	{
		return m_Pointer == other.m_Pointer;
	}

private:
	SimdUnOrderedVector<Joint>::Pointer* m_Pointer;
	SimdUnOrderedVector<Joint>* m_Joints;
};
