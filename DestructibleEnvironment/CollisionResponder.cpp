#include "pch.h"
#include "CollisionResponder.h"
#include "PhysicsObject.h"

static float CalculateS(const Vector3& n, const Vector3& r, const Matrix3 inertiaInverse)
{
	auto x = inertiaInverse * Vector3::Cross(r, n);
	return Vector3::Dot(n, Vector3::Cross(x, r));
}

static Vector3 GetNormal1To2(const PotentialCollision& coll, const Transform* body1Transform)
{
	return &coll.GetTransformOfPiercedFace() == body1Transform ?
		coll.GetWorldCollNormal() :
		-coll.GetWorldCollNormal();
}

Vector3 CollisionResponder::CalculateAverageNormal(const std::vector<PotentialCollision>& potColls)
{
	auto n = Vector3::Zero();
	auto body1Transform = &m_Body1->GetTransform();

	for (auto it = potColls.begin(); it != potColls.end(); it++)
		n += GetNormal1To2(*it, body1Transform);

	return n.Normalized();
}

bool CollisionResponder::CalculateCollisionPointAndNormal(const std::vector<PotentialCollision>& potColls, Vector3& finalNormal1To2, Vector3& finalPoint)
{
	auto weightSum = 0.0f;
	auto body1Transform = &m_Body1->GetTransform();

	finalNormal1To2 = Vector3::Zero();
	finalPoint = Vector3::Zero();

	for (auto it = potColls.begin(); it != potColls.end(); it++)
	{
		auto& coll = *it;

		auto n1To2 = GetNormal1To2(coll, body1Transform);
		auto& point = coll.GetCollPointWorld();

		auto v1 = m_Body1->WorldVelocityAt(point);
		auto v2 = m_Body2->WorldVelocityAt(point);

		auto vr = v2 - v1;

		auto signedImpact = Vector3::Dot(vr, n1To2);

		if (signedImpact < 0.0f)
		{
			auto impact = fabs(signedImpact);

			weightSum += impact;

			finalPoint += impact * point;
			finalNormal1To2 += impact * n1To2;
		}
	}

	if (weightSum > 0.0f)
	{
		finalPoint /= weightSum;
		finalNormal1To2.Normalize();
		return true;
	}
	return false;
}

float CollisionResponder::CalculateRequiredSeperation(const std::vector<PotentialCollision>& potColls, const Vector3& finalNormal1To2)
{
	auto maxSep = -1.0f;
	auto body2Transform = &m_Body2->GetTransform();

	for (auto it = potColls.begin(); it != potColls.end(); it++)
	{
		auto& coll = *it;

		// need to move body 2 in the direction of the normal so if the face
		// is from body 2, we need to move the face in the direction of the normal,
		// otherwise we need to move the edge.
		auto toSep = &coll.GetTransformOfPiercedFace() == body2Transform ?
			coll.CalculateRequiredSeperationWhenMovingFace(finalNormal1To2) :
			coll.CalculateRequiredSeperationWhenMovingEdge(finalNormal1To2);

		if (toSep > maxSep)
			maxSep = toSep;
	}

	return maxSep;
}

void CollisionResponder::CalculateResponse(const std::vector<PotentialCollision>& potColls, PhysicsObject& body1, PhysicsObject& body2)
{
	static constexpr float e = 0.5f; // TODO - get this from somewere else

	m_Body1 = &body1;
	m_Body2 = &body2;

	Vector3 collNormalWorld1To2;
	Vector3 collPointWorld;

	if (CalculateCollisionPointAndNormal(potColls, collNormalWorld1To2, collPointWorld))
	{
		auto v1 = body1.WorldVelocityAt(collPointWorld);
		auto v2 = body2.WorldVelocityAt(collPointWorld);

		auto vr = v2 - v1;

		auto signedImpact = Vector3::Dot(vr, collNormalWorld1To2);

		auto& t1 = body1.GetTransform();
		auto& t2 = body2.GetTransform();

		auto s1 = CalculateS(collNormalWorld1To2, collPointWorld - t1.GetPosition(), body1.GetInertiaInverseWorld());
		auto s2 = CalculateS(collNormalWorld1To2, collPointWorld - t2.GetPosition(), body2.GetInertiaInverseWorld());

		auto m1 = body1.GetMass();
		auto m2 = body2.GetMass();

		auto J = (-signedImpact * (e + 1.0f)) / ((1.0f / m1) + (1.0f / m2) + s1 + s2);

		auto impact = -signedImpact;

		auto v1N = MathU::Max(Vector3::Dot(v1, collNormalWorld1To2), 0.0f);
		auto v2N = MathU::Max(Vector3::Dot(v2, -collNormalWorld1To2), 0.0f);

		body1.AddImpulse(Impulse(-J * collNormalWorld1To2, collPointWorld, impact));
		body2.AddImpulse(Impulse(J * collNormalWorld1To2, collPointWorld, impact));

		auto toSep = CalculateRequiredSeperation(potColls, collNormalWorld1To2);

		body1.AddToRequiredToSeperate(-(v1N / (v1N + v2N)) * toSep * collNormalWorld1To2);
		body2.AddToRequiredToSeperate((v2N / (v1N + v2N)) * toSep * collNormalWorld1To2);
	}
	else
	{
		collNormalWorld1To2 = CalculateAverageNormal(potColls);
		auto toSep = CalculateRequiredSeperation(potColls, collNormalWorld1To2);

		body1.AddToRequiredToSeperate(-0.5f * toSep * collNormalWorld1To2);
		body2.AddToRequiredToSeperate(0.5f * toSep * collNormalWorld1To2);
	}
}