#pragma once
#include <atomic>
#include <vector>
#include <memory>
#include <thread>

class Shape;

class PhysicsEngine
{
public:

	auto& GetBodiesToBeAdded()
	{
		return m_BodiesToBeAdded;
	}

	auto& GetBodiesAdded()
	{
		return m_BodiesAdded;
	}

	void ClearSafeToSync()
	{
		m_SafeToSync = false;
	}

	bool IsSafeToSync()
	{
		return m_SafeToSync;
	}

	void StopRunning()
	{
		m_Running = false;
		m_SafeToSync = false; // do this so it doesnt get stuck waiting for flag to be cleared when game thread is finished.

		if (m_Thread.joinable())
			m_Thread.join();
	}

	void StartRunning();

private:
	void Run();

	void DoCollisionDetection();
	void UpdateBodies();
	void TransferBodiesAddedByGameThread();

	std::atomic<bool> m_Running = true;
	std::atomic<bool> m_SafeToSync = false;

	// the game thread fills this during collision detection, i.e. the sync phase. The bodies
	// are then added into main collection during updateBodies()
	std::vector<Shape*> m_BodiesToBeAdded;

	// this is filled with the bodies added by the physics engine (due to splits) during UpdateBodies().
	// the game thread creates proxies and clears the list at the next sync phase
	std::vector<Shape*> m_BodiesAdded;

	std::vector<std::unique_ptr<Shape>> m_Bodies;

	std::thread m_Thread;
};
