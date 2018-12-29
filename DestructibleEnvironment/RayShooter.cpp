#include "pch.h"
#include "RayShooter.h"
#include "World.h"
#include "Physics.h"
#include "Camera.h"
#include "DynamicBodyProxy.h"

void RayShooter::OnPhysicsWorldUpdated()
{
	m_LeftMouseButton.Update();
	if (m_LeftMouseButton.IsJustDown())
	{
		auto& w = GetWorld();

		auto mousePos = w.GetInput().GetMousePosition();
		auto ray = w.GetRenderer().GetActiveCamera().ScreenPointToRay(mousePos);

		auto hit = w.GetPhysics().RayCast(ray);

		if (hit.Hit())
		{
			auto body = hit.GetHitObject()->As<DynamicBodyProxy>();
			if (body)
			{
				body->AddImpulse(Impulse(ray.GetDirection(),
					hit.GetHitPoint(),
					1000.0f));
			}
		}
	}
}

void RayShooter::Awake()
{
	auto& w = GetWorld();

	w.GetPhysics().AddOnPhysicsUpdatedListener(*this);
	m_LeftMouseButton = InputChannelWrapper(w.GetInput().GetMouseChannel(1));
}