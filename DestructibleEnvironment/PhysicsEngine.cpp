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
		while (m_SafeToSync)
		{
			// collision detection will be expensive so execution shoulnt get to here
		}

		m_Time.WaitForNextUpdateTime();

		UpdateBodies();

		m_SafeToSync = true;

		DoCollisionDetection();
	}
}

void PhysicsEngine::DoCollisionDetection()
{
	auto dynamicCount = m_DynamicBodies.size();
	auto staticCount = m_StaticBodies.size();

	m_CollisionResponder.Reset();

	for (auto i = 0U; i < dynamicCount; i++)
	{
		auto& bodyi = *m_DynamicBodies[i];

		for (auto j = i + 1; j < dynamicCount; j++)
		{
			if (m_CollisionDetector.FindCollision(bodyi, *m_DynamicBodies[j], m_CollData))
				m_CollisionResponder.CalculateResponse(m_CollData, bodyi, *m_DynamicBodies[j]);
		}

		for (auto j = 0U; j < staticCount; j++)
		{
			if (m_CollisionDetector.FindCollision(bodyi, *m_StaticBodies[j], m_CollData))
				m_CollisionResponder.CalculateResponse(m_CollData, bodyi, *m_StaticBodies[j]);
		}
	}
}

void PhysicsEngine::UpdateBodies()
{
	ExecuteGameToPhysicsActions();

	m_Splits.clear();
	
	for (auto it = m_DynamicBodies.begin(); it != m_DynamicBodies.end(); it++)
		(*it)->Update(m_Splits);
	
	ProcessSplits();
}

void PhysicsEngine::ProcessSplits()
{
	static auto constexpr doSplits = true;

	if (doSplits && (m_DynamicBodies.size() < 10))
	{
		for (auto it = m_Splits.begin(); it != m_Splits.end(); it++)
		{
			auto& s = *it;
		
			auto newBody = std::unique_ptr<Rigidbody>(new Rigidbody()); // from pool
		
			auto& toSplit = *s.ToSplit;
		
			if (toSplit.Split(s.CauseImpulse->WorldCollisionPoint, *newBody))
			{
				newBody->CopyVelocity(toSplit);
		
				newBody->CalculateMotionProperties();
				toSplit.CalculateMotionProperties();
		
				m_BodiesAdded.emplace_back(newBody.get());
				m_DynamicBodies.emplace_back(std::move(newBody));
			}
			else
			{
				// return new body to pool
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