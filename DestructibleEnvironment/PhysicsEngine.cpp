#include "pch.h"
#include "PhysicsEngine.h"
#include "PhysicsTime.h"

void PhysicsEngine::SimulateOneTimeStep()
{
	ApplyExternalForcesAndImpulses();
	FindContacts();
	SatisfyConstraints();
	UpdateBodies();
}

RayCastHit<CompoundShape> PhysicsEngine::RayCast(const Ray& r) const
{
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
	m_Solver.Solve(m_Collision.GetContactConstraints(), m_Collision.GetManifolds());
	m_Collision.StoreAccImpulses();
}

void PhysicsEngine::UpdateBodies()
{
	m_Splits.clear();
	
	for (auto it = m_DynamicBodies.begin(); it != m_DynamicBodies.end(); it++)
	{
		auto& b = **it;
		b.UpdatePosition();
	}
}