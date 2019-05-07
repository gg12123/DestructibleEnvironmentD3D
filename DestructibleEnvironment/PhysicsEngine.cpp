#include "pch.h"
#include "PhysicsEngine.h"
#include "PhysicsTime.h"

void PhysicsEngine::SimulateOneTimeStep()
{
	ApplyExternalForcesAndImpulses();
	FindConstraints();
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
	{
		if (b->IsAwake())
			b->ApplyExternalForcesAndImpulses();
	}
}

void PhysicsEngine::FindConstraints()
{
	m_Constraints.FindConstraints(m_StaticBodies, m_DynamicBodies);
}

void PhysicsEngine::SatisfyConstraints()
{
	m_Solver.Solve(m_Constraints.GetContactConstraints(), m_Constraints.GetManifolds(),
		m_Constraints.GetJoints(), m_Constraints.GetIslands());

	m_Constraints.StoreAccImpulses();
}

void PhysicsEngine::UpdateBodies()
{
	for (auto& b : m_DynamicBodies)
	{
		if (b->IsAwake())
			b->UpdatePosition();
	}
}