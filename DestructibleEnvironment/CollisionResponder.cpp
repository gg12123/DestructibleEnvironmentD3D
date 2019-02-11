#include "pch.h"
#include "CollisionResponder.h"
#include "PhysicsObject.h"

static float CalculateS(const Vector3& n, const Vector3& r, const Matrix3 inertiaInverse)
{
	auto x = inertiaInverse * Vector3::Cross(r, n);
	return Vector3::Dot(n, Vector3::Cross(x, r));
}

bool CollisionResponder::CalculateSeperation1To2(std::vector<FaceCollision>& faceColls, Vector3& sep1To2) const
{
	static constexpr float allowableOverlap = 0.001f;

	auto biggestMag = 0.0f;

	for (auto& coll : faceColls)
	{
		auto x = coll.CalculateSeperationVectors(*m_Body2);

		if (x.GetMag() > biggestMag)
		{
			biggestMag = x.GetMag();
			sep1To2 = biggestMag * x.GetDir();
		}
	}
	return biggestMag > allowableOverlap;
}

Vector3 AverageInterPoint(const std::vector<EdgeFaceIntersection>& inters)
{

}

bool CollisionResponder::CalculateCollisionPoint(const std::vector<EdgeFaceIntersection>& inters, const Vector3& normal1To2, Vector3& finalPoint)
{
	auto weightSum = 0.0f;
	auto body1Transform = &m_Body1->GetTransform();

	finalPoint = Vector3::Zero();

	for (auto it = inters.begin(); it != inters.end(); it++)
	{
		auto& inter = *it;

		auto& point = inter.GetIntPoint();

		auto v1 = m_Body1->WorldVelocityAt(point);
		auto v2 = m_Body2->WorldVelocityAt(point);

		auto vr = v2 - v1;

		auto signedImpact = Vector3::Dot(vr, normal1To2);

		if (signedImpact < 0.0f)
		{
			auto impact = fabs(signedImpact);

			weightSum += impact;
			finalPoint += impact * point;
		}
	}

	if (weightSum > 0.0f)
	{
		finalPoint /= weightSum;
		return true;
	}
	return false;
}

void CollisionResponder::CalculateResponse(std::vector<FaceCollision>& faceColls,
						const std::vector<EdgeFaceIntersection>& inters,
						PhysicsObject& body1, PhysicsObject& body2)
{
	static constexpr float e = 0.5f; // TODO - get this from somewere else

	m_Body1 = &body1;
	m_Body2 = &body2;

	Vector3 reqToSeperate;
	if (!CalculateSeperation1To2(faceColls, reqToSeperate))
		return;

	auto collNormalWorld1To2 = reqToSeperate.Normalized();
	auto toSep = reqToSeperate.Magnitude();

	Vector3 collPointWorld;
	if (CalculateCollisionPoint(inters, collNormalWorld1To2, collPointWorld))
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

		body1.AddImpulse(Impulse(-J * collNormalWorld1To2, collPointWorld, impact));
		body2.AddImpulse(Impulse(J * collNormalWorld1To2, collPointWorld, impact));
	}
	else
	{
		collPointWorld = AverageInterPoint(inters);
	}

	body1.AddContact(ContactManifold(collPointWorld, -collNormalWorld1To2));
	body2.AddContact(ContactManifold(collPointWorld, collNormalWorld1To2));
}