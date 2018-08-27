#include "DynamicBodyProxy.h"
#include "World.h"

Shape & DynamicBodyProxy::RegisterWithPhysics()
{
	return GetWorld().GetPhysics().AddDynamicRigidbody(*this);
}