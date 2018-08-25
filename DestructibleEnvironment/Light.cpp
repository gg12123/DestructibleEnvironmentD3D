#include "pch.h"
#include "Light.h"
#include "World.h"

void Light::Awake()
{
	GetWorld().GetRenderer().AddLight(*this);
}