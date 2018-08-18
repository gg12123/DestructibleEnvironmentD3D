#include "pch.h"
#include "PhysicsEngine.h"

void PhysicsEngine::StartRunning()
{
	// create thread here
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
	TransferBodiesAddedByGameThread();

	// add forces and do required splits, putting all new bodies into m_BodiesAdded
}

void PhysicsEngine::TransferBodiesAddedByGameThread()
{
	for (auto it = m_BodiesToBeAdded.begin(); it != m_BodiesToBeAdded.end(); it++)
		m_Bodies.push_back(std::unique_ptr<Shape>(*it));

	m_BodiesToBeAdded.clear();
}