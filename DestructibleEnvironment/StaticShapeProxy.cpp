#include "StaticShapeProxy.h"
#include "World.h"

Shape & StaticShapeProxy::RegisterWithPhysics()
{
	return GetWorld().GetPhysics().AddStaticRigidbody(*this);
}