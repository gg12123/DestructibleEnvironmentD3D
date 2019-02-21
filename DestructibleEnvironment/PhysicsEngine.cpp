#include "pch.h"
#include "PhysicsEngine.h"
#include "PhysicsTime.h"

void PhysicsEngine::StartRunning()
{
	if (!m_Thread.joinable())
		m_Thread = std::thread(&PhysicsEngine::Run, this);
}

void PhysicsEngine::Run()
{
	m_Time.SetFixedDeltaTime(PhysicsTime::FixedDeltaTime);
	m_Time.Start();

	while (m_Running)
	{
		//while (m_SafeToSync)
		//{
		//	// Collision detection will be expensive so execution shouln't be here for long.
		//}
		//
		//m_Time.WaitForNextUpdateTime();
		//
		//UpdateBodies();
		//
		//m_SafeToSync = true;
		//
		//DoCollisionDetectionResponse();

		ApplyExternalForces();

		m_SafeToSync = true;
		DoCollisionDetectionResponse();
		while (m_SafeToSync)
			;

		UpdateBodies();

		m_Time.WaitForNextUpdateTime();
	}
}

RayCastHit<CompoundShape> PhysicsEngine::RayCast(const Ray& r) const
{
	assert(m_SafeToSync);

	auto cast = UpdatableRayCast<CompoundShape>(r);

	cast.Update(m_DynamicBodies);
	cast.Update(m_StaticBodies);

	return cast.ToRayCastHit();
}

void PhysicsEngine::ApplyExternalForces() const
{
	for (auto& b : m_DynamicBodies)
		b->ApplyExternalForces();
}

void PhysicsEngine::DoCollisionDetectionResponse()
{
	m_Collision.DetectAndRespond(m_StaticBodies, m_DynamicBodies);
}

void PhysicsEngine::UpdateBodies()
{
	ExecuteGameToPhysicsActions();

	m_Splits.clear();
	
	for (auto it = m_DynamicBodies.begin(); it != m_DynamicBodies.end(); it++)
	{
		auto& b = **it;

		b.UpdatePosition(m_Splits);

		// Must fully re-calculate the transform so that it doesnt re-calculate during collision
		// detection whilst it may be getting accsesed from the game thread.
		b.GetTransform().ReCalculateIfDirty();
	}
	
	ProcessSplits();
}

void PhysicsEngine::ProcessSplits()
{
	static auto constexpr doSplits = false;

	if (doSplits)
	{
		for (auto it = m_Splits.begin(); it != m_Splits.end(); it++)
		{
			auto& s = *it;
			auto& toSplit = *s.ToSplit;

			m_NewBodiesFromSplit.clear();
			m_ShapeDivider.DivideShape(1, m_NewBodiesFromSplit, toSplit);

			for (auto newBody : m_NewBodiesFromSplit)
			{
				if (newBody != &toSplit)
				{
					m_BodiesAdded.emplace_back(newBody);
					m_DynamicBodies.emplace_back(std::unique_ptr<Rigidbody>(newBody));
				}

				newBody->CopyVelocity(toSplit);
				newBody->CalculateMotionProperties();
				newBody->GetTransform().ReCalculateIfDirty();
			}
		}
	}
	m_Splits.clear();
}

void PhysicsEngine::ExecuteGameToPhysicsActions()
{
	for (auto it = m_GameToPhysicsActions.begin(); it != m_GameToPhysicsActions.end(); it++)
		(*it)->Apply(*this);

	m_GameToPhysicsActions.clear();
}