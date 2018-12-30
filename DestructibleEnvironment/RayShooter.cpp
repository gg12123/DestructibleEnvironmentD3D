#include "pch.h"
#include "RayShooter.h"
#include "World.h"
#include "Physics.h"
#include "Camera.h"
#include "DynamicBodyProxy.h"
#include "PlaneMesh.h"

void RayShooter::OnPhysicsWorldUpdated()
{
	m_LeftMouseButton.Update();
	if (m_LeftMouseButton.IsJustDown())
	{
		auto& w = GetWorld();

		auto mousePos = w.GetInput().GetMousePosition();
		auto ray = w.GetRenderer().GetActiveCamera().ScreenPointToRay(mousePos);

		//auto plane = new PlaneMesh(Vector2(1.0f, 1.0f));
		//plane->GetTransform().SetPosition(ray.GetOrigin() + 10.0f * ray.GetDirection());
		//plane->GetTransform().SetRotation(Quaternion::LookRotation(-ray.GetDirection()));
		//w.RegisterEntity(std::unique_ptr<Entity>(plane));

		auto hit = w.GetPhysics().RayCast(ray);
		
		if (hit.Hit())
		{
			auto body = hit.GetHitObject()->As<DynamicBodyProxy>();
			if (body)
			{
				body->AddImpulse(Impulse(ray.GetDirection(),
					hit.GetHitPoint(),
					2000.0f));
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