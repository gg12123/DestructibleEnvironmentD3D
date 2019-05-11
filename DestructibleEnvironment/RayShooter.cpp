#include "pch.h"
#include "RayShooter.h"
#include "World.h"
#include "Physics.h"
#include "Camera.h"
#include "DynamicBodyProxy.h"
#include "PlaneMesh.h"

void RayShooter::Update()
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
				auto imp = Impulse(2.0f * ray.GetDirection(), hit.GetHitPoint());

				body->AddImpulse(imp);
				GetWorld().GetPhysics().DestructBody(*body, imp);
			}
		}
	}
}

void RayShooter::Awake()
{
	auto& w = GetWorld();

	w.RegisterEntityForUpdate(*this);
	m_LeftMouseButton = InputChannelWrapper(w.GetInput().GetMouseChannel(1));
}