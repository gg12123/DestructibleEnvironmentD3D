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

	// add forces and do required splits, putting all new bodies into m_BodiesAdded
}

void PhysicsEngine::ExecuteGameToPhysicsActions()
{
	for (auto it = m_GameToPhysicsActions.begin(); it != m_GameToPhysicsActions.end(); it++)
		(*it)->Apply(*this);

	m_GameToPhysicsActions.clear();
}