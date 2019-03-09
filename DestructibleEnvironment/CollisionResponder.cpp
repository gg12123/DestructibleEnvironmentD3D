#include "pch.h"
#include "CollisionResponder.h"
#include "PhysicsObject.h"
#include "Shape.h"
#include "PhysicsTime.h"

static float CalculateS(const Vector3& n, const Vector3& r, const Matrix3 inertiaInverse)
{
	auto x = inertiaInverse * Vector3::Cross(r, n);
	return Vector3::Dot(n, Vector3::Cross(x, r));
}

float CollisionResponder::CalculateImpulse(const ContactPlane& worldContact1To2, PhysicsObject& body1, PhysicsObject& body2)
{
	auto collNormalWorld1To2 = worldContact1To2.GetNormal();
	auto collPointWorld = worldContact1To2.GetPoint();

	auto v1 = body1.WorldVelocityAt(collPointWorld);
	auto v2 = body2.WorldVelocityAt(collPointWorld);

	auto& t1 = body1.GetTransform();
	auto& t2 = body2.GetTransform();

	auto s1 = CalculateS(collNormalWorld1To2, collPointWorld - t1.GetPosition(), body1.GetInertiaInverseWorld());
	auto s2 = CalculateS(collNormalWorld1To2, collPointWorld - t2.GetPosition(), body2.GetInertiaInverseWorld());

	auto m1 = body1.GetMass();
	auto m2 = body2.GetMass();

	static constexpr auto beta = 0.1f;
	static constexpr auto slop = 0.01f;
	auto vBias = (beta / PhysicsTime::FixedDeltaTime) * MathU::Max(worldContact1To2.GetPeneration() - slop, 0.0f);

	auto vr = v2 - v1;
	auto J = (-Vector3::Dot(vr, collNormalWorld1To2) + vBias) / ((1.0f / m1) + (1.0f / m2) + s1 + s2);

	return J;
}