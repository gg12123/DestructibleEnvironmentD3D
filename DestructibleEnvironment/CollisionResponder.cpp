#include "pch.h"
#include "CollisionResponder.h"
#include "PhysicsObject.h"

static float CalculateS(const Vector3& n, const Vector3& r, const Matrix3 inertiaInverse)
{
	auto x = inertiaInverse * Vector3::Cross(r, n);
	return Vector3::Dot(n, Vector3::Cross(x, r));
}

void CollisionResponder::CalculateResponse(const CollisionData& collData, PhysicsObject& body1, PhysicsObject& body2)
{
	static constexpr float e = 0.5f; // TODO - get this from somewere else

	auto& collPointWorld = collData.Position;
	auto& collNormalWorld = collData.Normal1To2;
	auto pen = collData.Penetration;

	auto v1 = body1.WorldVelocityAt(collPointWorld);
	auto v2 = body2.WorldVelocityAt(collPointWorld);

	auto vr = v2 - v1;

	auto signedImpact = Vector3::Dot(vr, collNormalWorld);

	if (signedImpact < 0.0f)
	{
		auto& t1 = body1.GetTransform();
		auto& t2 = body2.GetTransform();

		auto s1 = CalculateS(collNormalWorld, collPointWorld - t1.GetPosition(), body1.GetInertiaInverseWorld());
		auto s2 = CalculateS(collNormalWorld, collPointWorld - t2.GetPosition(), body2.GetInertiaInverseWorld());

		auto m1 = body1.GetMass();
		auto m2 = body2.GetMass();

		auto J = (-signedImpact * (e + 1.0f)) / ((1.0f / m1) + (1.0f / m2) + s1 + s2);

		auto impact = -signedImpact;

		auto v1N = MathUtils::Max(Vector3::Dot(v1, collNormalWorld), 0.0f);
		auto v2N = MathUtils::Max(Vector3::Dot(v2, -collNormalWorld), 0.0f);

		auto& impulse1 = *m_ImpulseDataPool->Recycle();
		auto& impulse2 = *m_ImpulseDataPool->Recycle();

		impulse1.Impact = impact;
		impulse1.WorldCollisionPoint = collPointWorld;
		impulse1.WorldImpulse = -J * collNormalWorld;

		impulse2.Impact = impact;
		impulse2.WorldCollisionPoint = collPointWorld;
		impulse2.WorldImpulse = J * collNormalWorld;

		body1.AddImpulse(impulse1);
		body2.AddImpulse(impulse2);

		body1.AddToRequiredToSeperate(-(v1N / (v1N + v2N)) * pen * collNormalWorld);
		body2.AddToRequiredToSeperate((v2N / (v1N + v2N)) * pen * collNormalWorld);
	}
	else
	{
		body1.AddToRequiredToSeperate(-0.5f * pen * collNormalWorld);
		body2.AddToRequiredToSeperate(0.5f * pen * collNormalWorld);
	}
}