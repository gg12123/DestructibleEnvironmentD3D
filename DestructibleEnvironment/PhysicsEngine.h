#pragma once
#include <atomic>
#include <vector>

class Physics;
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

private:
	void Run();

	void DoCollisionDetection();
	void UpdateBodies();
	void TransferBodiesAddedByGameThread();

	std::atomic<bool> m_Running = true;
	std::atomic<bool> m_SafeToSync = false;

	Physics *m_PhysicsProxy;

	std::vector<Shape*> m_BodiesToBeAdded; // the game thread fills this during collision detection, so it is safe to access during updateBodies()
	std::vector<Shape*> m_BodiesAdded;

	std::vector<Shape*> m_Bodies;
};
