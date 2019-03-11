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
		ApplyExternalForcesAndImpulses();

		m_SafeToSync = true;
		FindContacts();
		while (m_SafeToSync)
			;

		SatisfyConstraints();

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

void PhysicsEngine::ApplyExternalForcesAndImpulses() const
{
	for (auto& b : m_DynamicBodies)
		b->ApplyExternalForcesAndImpulses();
}

void PhysicsEngine::FindContacts()
{
	m_Collision.FindContacts(m_StaticBodies, m_DynamicBodies);
}

void PhysicsEngine::SatisfyConstraints()
{
	m_Solver.Solve(m_Collision.GetContactConstraints());
}

void PhysicsEngine::UpdateBodies()
{
	ExecuteGameToPhysicsActions();

	m_Splits.clear();
	
	for (auto it = m_DynamicBodies.begin(); it != m_DynamicBodies.end(); it++)
	{
		auto& b = **it;

		b.UpdatePosition();

		if (b.IsSplit())
		{
			m_Splits.emplace_back(SplitInfo(b, b.GetSplittingImpulse()));
			b.ClearSplit();
		}
		else
		{
			// Only add the object to collision if it is not split. Split results
			// are added after the split.
			m_Collision.AddObject(b);
		}
	}
	
	ProcessSplits();
}

void PhysicsEngine::ProcessSplits()
{
	static auto constexpr doSplits = true;

	if (doSplits)
	{
		for (auto& data : m_Splits)
		{
			auto& toSplit = *data.ToSplit;

			m_NewBodiesFromDestruct.clear();
			m_ShapeDestructor.Destruct(toSplit, data.CauseImpulse, m_NewBodiesFromDestruct);

			for (auto newBody : m_NewBodiesFromDestruct)
			{
				if (newBody != &toSplit)
				{
					m_BodiesAdded.emplace_back(newBody);
					m_DynamicBodies.emplace_back(std::unique_ptr<Rigidbody>(newBody));
				}

				newBody->CopyVelocity(toSplit);
				newBody->CopyDrag(toSplit);
				newBody->CalculateMassProperties();

				m_Collision.AddObject(*newBody);
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