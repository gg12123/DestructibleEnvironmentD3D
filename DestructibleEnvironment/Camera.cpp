#include "pch.h"
#include "Camera.h"
#include "World.h"

void Camera::Awake()
{
	GetWorld().GetRenderer().SetCamera(*this);
}