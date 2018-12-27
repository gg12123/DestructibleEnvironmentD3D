#include "RayShooter.h"
#include "World.h"
#include "Physics.h"

void RayShooter::OnPhysicsWorldUpdated()
{
	m_LeftMouseButton.Update();
	if (m_LeftMouseButton.IsJustDown())
	{
		// shoot the ray
	}
}

void RayShooter::Awake()
{
	auto& w = GetWorld();

	w.GetPhysics().AddOnPhysicsUpdatedListener(*this);
	m_LeftMouseButton = InputChannelWrapper(w.GetInput().GetMouseChannel(0));
}