#include "pch.h"
#include "StaticShapeProxy.h"
#include "World.h"

CompoundShape & StaticShapeProxy::RegisterWithPhysics()
{
	m_StaticBody = &GetWorld().GetPhysics().AddStaticRigidbody(*this);
	return *m_StaticBody;
}