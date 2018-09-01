#include "pch.h"
#include "DynamicBodyProxy.h"
#include "World.h"
#include "Rigidbody.h"

DynamicBodyProxy::DynamicBodyProxy(Shape& body) : ShapeProxy(body)
{
}

Shape & DynamicBodyProxy::RegisterWithPhysics()
{
	return GetWorld().GetPhysics().AddDynamicRigidbody(*this);
}