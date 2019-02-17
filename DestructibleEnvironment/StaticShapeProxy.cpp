#include "pch.h"
#include "StaticShapeProxy.h"
#include "World.h"

CompoundShape & StaticShapeProxy::RegisterWithPhysics()
{
	return GetWorld().GetPhysics().AddStaticRigidbody(*this);
}