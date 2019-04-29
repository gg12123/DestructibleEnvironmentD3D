#pragma once
#include <atomic>
#include <vector>
#include <memory>
#include <thread>
#include "Constants.h"
#include "Rigidbody.h"
#include "StaticBody.h"
#include "SplitInfo.h"
#include "Collision.h"
#include "FixedTimeStepTime.h"
#include "ShapeDestructor.h"
#include "RayCasting.h"
#include "SequentialImpulsesSolver.h"

class PhysicsEngine
{
public:
	PhysicsEngine()
	{
		m_DynamicBodies.reserve(Constants::MaxNumShapes);
	}

	auto& GetBodiesAdded()
	{
		return m_BodiesAdded;
	}

	const auto& GetDynamicBodies() const
	{
		return m_DynamicBodies;
	}

	const auto& GetStaticBodies() const
	{
		return m_StaticBodies;
	}

	void SimulateOneTimeStep();

	RayCastHit<CompoundShape> RayCast(const Ray& r) const;

	void AddDynamicBody(std::unique_ptr<Rigidbody>&& body)
	{
		m_DynamicBodies.emplace_back(std::move(body));
	}

	void AddStaticBody(std::unique_ptr<StaticBody>&& body)
	{
		m_StaticBodies.emplace_back(std::move(body));
	}

private:
	void FindContacts();
	void UpdateBodies();
	void SatisfyConstraints();
	void ApplyExternalForcesAndImpulses() const;

	// This is filled with the bodies added by the physics engine (due to splits) during UpdateBodies().
	// the game thread creates proxies and clears the list at the next sync phase
	std::vector<Rigidbody*> m_BodiesAdded;

	std::vector<std::unique_ptr<Rigidbody>> m_DynamicBodies;
	std::vector<std::unique_ptr<StaticBody>> m_StaticBodies;

	Collision m_Collision;
	SequentialImpulsesSolver m_Solver;

	std::vector<SplitInfo> m_Splits;
	ShapeDestructor<Rigidbody> m_ShapeDestructor;
	std::vector<Rigidbody*> m_NewBodiesFromDestruct;
};
