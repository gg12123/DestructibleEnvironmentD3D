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
			// Collision detection will be expensive so execution shouln't be here for long.
		}

		m_Time.WaitForNextUpdateTime();

		UpdateBodies();

		m_SafeToSync = true;

		DoCollisionDetectionResponse();
	}
}

void PhysicsEngine::DoCollisionDetectionResponse(PhysicsObject& body1, PhysicsObject& body2)
{
	if (m_CollisionDetector.FindCollision(body1, body2, m_FaceCollisions, m_Intersections))
		m_CollisionResponder.CalculateResponse(m_FaceCollisions, m_Intersections, body1, body2);
}

void PhysicsEngine::DoCollisionDetectionResponse()
{
	auto dynamicCount = m_DynamicBodies.size();
	auto staticCount = m_StaticBodies.size();

	// TODO - partition

	for (auto i = 0U; i < dynamicCount; i++)
	{
		auto& bodyi = *m_DynamicBodies[i];

		for (auto j = i + 1; j < dynamicCount; j++)
			DoCollisionDetectionResponse(bodyi, *m_DynamicBodies[j]);

		for (auto j = 0U; j < staticCount; j++)
			DoCollisionDetectionResponse(bodyi, *m_StaticBodies[j]);
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

	if (doSplits && (m_DynamicBodies.size() < 2))
	{
		for (auto it = m_Splits.begin(); it != m_Splits.end(); it++)
		{
			auto& s = *it;
		
			auto& toSplit = *s.ToSplit;

			m_NewBodiesFromSplit.clear();
			m_Splitter.Split(s.CauseImpulse.WorldCollisionPoint, s.CauseImpulse.WorldImpulse.Normalized(), toSplit, m_NewBodiesFromSplit);
		
			// TODO - this is a bit messy. The first entry in the list
			// is the original shape so no need to add it to the world etc.
			for (auto i = 1U; i < m_NewBodiesFromSplit.size(); i++)
			{
				auto newBody = m_NewBodiesFromSplit[i];
				m_BodiesAdded.emplace_back(newBody);
				m_DynamicBodies.emplace_back(std::unique_ptr<Rigidbody>(newBody));
			}

			// The original will copy velocity from itself but that doesnt really matter
			for (auto it = m_NewBodiesFromSplit.begin(); it != m_NewBodiesFromSplit.end(); it++)
			{
				auto& newBody = **it;
				newBody.CopyVelocity(toSplit);
				newBody.CalculateMotionProperties();
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