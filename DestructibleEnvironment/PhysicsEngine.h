#pragma once
#include <atomic>
#include <vector>
#include <memory>
#include <thread>
#include "Constants.h"
#include "GameThreadToPhysicsThreadAction.h"
#include "Rigidbody.h"
#include "StaticBody.h"
#include "SplitInfo.h"
#include "Collision.h"
#include "FixedTimeStepTime.h"
#include "ShapeChunkTaker.h"
#include "RayCasting.h"
#include "SequentialImpulsesSolver.h"

class PhysicsEngine
{
public:
	PhysicsEngine()
	{
		m_DynamicBodies.reserve(Constants::MaxNumShapes);
	}

	auto& GetGameToPhysicsActions()
	{
		return m_GameToPhysicsActions;
	}

	auto& GetBodiesAdded()
	{
		return m_BodiesAdded;
	}

	auto& GetDynamicBodies()
	{
		return m_DynamicBodies;
	}

	auto& GetStaticBodies()
	{
		return m_StaticBodies;
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
		m_SafeToSync = false; // Do this so it doesnt get stuck waiting for flag to be cleared when game thread is finished.

		if (m_Thread.joinable())
			m_Thread.join();
	}

	void StartRunning();

	RayCastHit<CompoundShape> RayCast(const Ray& r) const;

private:
	void Run();

	void FindContacts();
	void UpdateBodies();
	void SatisfyConstraints();
	void ExecuteGameToPhysicsActions();
	void ProcessSplits();
	void ApplyExternalForcesAndImpulses() const;

	std::atomic<bool> m_Running = true;
	std::atomic<bool> m_SafeToSync = false;

	// The game thread fills this during collision detection, i.e. the sync phase.
	// the actions are executed at the start of UpdateBodies().
	std::vector<std::unique_ptr<IGameTheadToPhysicsThreadAction>> m_GameToPhysicsActions;

	// This is filled with the bodies added by the physics engine (due to splits) during UpdateBodies().
	// the game thread creates proxies and clears the list at the next sync phase
	std::vector<Rigidbody*> m_BodiesAdded;

	std::vector<std::unique_ptr<Rigidbody>> m_DynamicBodies;
	std::vector<std::unique_ptr<StaticBody>> m_StaticBodies;

	Collision m_Collision;
	SequentialImpulsesSolver m_Solver;

	std::vector<Rigidbody*> m_Splits;
	ShapeChunkTaker<Rigidbody> m_ShapeChunker;
	std::vector<Rigidbody*> m_NewBodiesFromSplit;

	FixedTimeStepTime m_Time;

	std::thread m_Thread;
};
