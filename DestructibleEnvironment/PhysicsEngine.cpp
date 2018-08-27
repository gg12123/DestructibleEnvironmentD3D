#include "pch.h"
#include "PhysicsEngine.h"

void PhysicsEngine::StartRunning()
{
	if (!m_Thread.joinable())
		m_Thread = std::thread(&PhysicsEngine::Run, this);
}

void PhysicsEngine::Run()
{
	while (m_Running)
	{
		while (m_SafeToSync) // and wait for fixed time step to elapse
		{
			// collision detection will be expensive so execution shoulnt get to here
		}

		UpdateBodies();

		m_SafeToSync = true;

		DoCollisionDetection();
	}
}

void PhysicsEngine::DoCollisionDetection()
{

}

void PhysicsEngine::UpdateBodies()
{
	ExecuteGameToPhysicsActions();

	m_Splits.clear();

	for (auto it = m_DynamicBodies.begin(); it != m_DynamicBodies.end; it++)
		(*it)->Update(m_Splits);

	ProcessSplits();
}

void PhysicsEngine::ProcessSplits()
{
	for (auto it = m_Splits.begin(); it != m_Splits.end(); it++)
	{
		auto& s = *it;

		auto newBody = std::unique_ptr<Rigidbody>(new Rigidbody()); // from pool
		newBody->CopyVelocity(*s.ToSplit);

		s.ToSplit->Split(s.CauseImpulse->WorldCollisionPoint, *newBody);

		m_BodiesAdded.emplace_back(newBody.get());
		m_DynamicBodies.emplace_back(std::move(newBody));
	}
	m_Splits.clear();
}

void PhysicsEngine::ExecuteGameToPhysicsActions()
{
	for (auto it = m_GameToPhysicsActions.begin(); it != m_GameToPhysicsActions.end(); it++)
		(*it)->Apply(*this);

	m_GameToPhysicsActions.clear();
}