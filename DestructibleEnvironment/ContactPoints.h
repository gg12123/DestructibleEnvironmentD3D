#pragma once
#include "PhysicsObject.h"
#include "Shape.h"
#include "CollisionData.h"
#include "CollisionResponder.h"
#include "Face.h"
#include "PhysicsTime.h"

class ContactConstraint
{
private:
	static float CalculateS(const Vector3& n, const Vector3& r, const Matrix3 inertiaInverse)
	{
		auto x = inertiaInverse * Vector3::Cross(r, n);
		return Vector3::Dot(n, Vector3::Cross(x, r));
	}

public:
	ContactConstraint(PhysicsObject& body1, PhysicsObject& body2, const Vector3& direction, const Vector3& point)
		: m_Body1(&body1), m_Body2(&body2), m_Direction(direction), m_Point(point), m_AcumulatedImpulse(0.0f)
	{
		auto& t1 = body1.GetTransform();
		auto& t2 = body2.GetTransform();

		auto s1 = CalculateS(m_Direction, m_Point - t1.GetPosition(), body1.GetInertiaInverseWorld());
		auto s2 = CalculateS(m_Direction, m_Point - t2.GetPosition(), body2.GetInertiaInverseWorld());

		m_Denom = (body1.GetInvMass() + body2.GetInvMass() + s1 + s2);
	}

	auto& GetBody1() const
	{
		return *m_Body1;
	}

	auto& GetBody2() const
	{
		return *m_Body2;
	}

	const auto& GetPoint() const
	{
		return m_Point;
	}

	const auto& GetDirection() const
	{
		return m_Direction;
	}

	auto GetDenom() const
	{
		return m_Denom;
	}

	float GetAccumulatedImpulse() const
	{
		return m_AcumulatedImpulse;
	}

	void SetAccumulatedImpulse(float val)
	{
		m_AcumulatedImpulse = val;
	}

	float CalculateRelativeVelocity() const
	{
		auto v1 = m_Body1->WorldVelocityAt(m_Point);
		auto v2 = m_Body2->WorldVelocityAt(m_Point);
		auto vr = v2 - v1;

		return Vector3::Dot(vr, m_Direction);
	}

protected:
	void OrientateDirection(const Vector3& refDir)
	{
		m_Direction = m_Direction.InDirectionOf(refDir);
	}

	void ApplyImpulse(float J) const
	{
		m_Body1->ApplyImpulse(Impulse(-J * m_Direction, m_Point, J));
		m_Body2->ApplyImpulse(Impulse(J * m_Direction, m_Point, J));
	}

private:
	PhysicsObject * m_Body1;
	PhysicsObject * m_Body2;
	Vector3 m_Direction;
	Vector3 m_Point;
	float m_Denom;
	float m_AcumulatedImpulse;
};

class NormalContactConstraint : public ContactConstraint
{
private:
	float CalculateCurrentImpulse() const
	{
		static constexpr auto beta = 0.1f;
		static constexpr auto slop = 0.01f;
		auto vBias = (beta / PhysicsTime::FixedDeltaTime) * MathU::Max(m_Penetration - slop, 0.0f);

		return (-CalculateRelativeVelocity() + vBias) / GetDenom();
	}

public:
	NormalContactConstraint(const Shape& s1, const Shape& s2, const ContactPlane& contactPlane) :
		ContactConstraint(*s1.GetOwner().ToPhysicsObject(), *s2.GetOwner().ToPhysicsObject(),
			contactPlane.GetNormal(), contactPlane.GetPoint()),
		m_Penetration(contactPlane.GetPeneration())
	{
		auto refN = GetBody2().GetTransform().ToWorldPosition(s2.GetCentre()) - GetBody1().GetTransform().ToWorldPosition(s1.GetCentre());
		OrientateDirection(refN);
	}

	float ApplyNextImpulse()
	{
		auto prevAccImpulse = GetAccumulatedImpulse();

		auto delta = CalculateCurrentImpulse();
		SetAccumulatedImpulse(MathU::Max(prevAccImpulse + delta, 0.0f));

		auto change = GetAccumulatedImpulse() - prevAccImpulse;
		ApplyImpulse(change);

		return change;
	}

private:
	float m_Penetration;
};

class FrictionContactConstraint : ContactConstraint
{
private:
	float CalculateCurrentImpulse() const
	{
		return (-CalculateRelativeVelocity()) / GetDenom();
	}

public:
	FrictionContactConstraint(const Shape& s1, const Shape& s2, const Vector3& manCentre, const Vector3& dir) :
		ContactConstraint(*s1.GetOwner().ToPhysicsObject(), *s2.GetOwner().ToPhysicsObject(), dir, manCentre)
	{
	}

	void ApplyNextImpulse(float Jn)
	{
		auto prevAccImpulse = GetAccumulatedImpulse();

		auto delta = CalculateCurrentImpulse();
		
		static constexpr float mu = 0.5f;
		SetAccumulatedImpulse(MathU::Clamp(prevAccImpulse + delta, -mu * Jn, mu * Jn));

		auto change = GetAccumulatedImpulse() - prevAccImpulse;
		ApplyImpulse(change);
	}
};

class ContactManifold
{
public:
	ContactManifold(const Shape& s1, const Shape& s2, int start, int end, const Vector3& dirA, const Vector3& dirB, const Vector3& centre) :
		m_Start(start), m_End(end), m_FrictionA(s1, s2, centre, dirA), m_FrictionB(s1, s2, centre, dirB)
	{
	}

	void ApplyImpulses(std::vector<NormalContactConstraint>& contactPoints)
	{
		auto aveJn = 0.0f;
		for (auto i = m_Start; i < m_End; i++)
		{
			aveJn += contactPoints[i].ApplyNextImpulse();
		}

		if (aveJn > 0.0f)
		{
			aveJn /= static_cast<float>(m_End - m_Start);
			m_FrictionA.ApplyNextImpulse(aveJn);
			m_FrictionB.ApplyNextImpulse(aveJn);
		}
	}

private:
	int m_Start;
	int m_End;
	FrictionContactConstraint m_FrictionA;
	FrictionContactConstraint m_FrictionB;
};

class ContactPointFinder
{
private:
	bool PointIsInsideShape(const Shape& shape, const Vector3& worldPoint) const
	{
		auto localPoint = shape.GetOwner().GetTransform().ToLocalPosition(worldPoint);

		for (auto f : shape.GetFaces())
		{
			if (Vector3::Dot(localPoint - f->GetPlaneP0(), f->GetNormal()) > 0.0f)
				return false;
		}
		return true;
	}

	void FindConstraints(std::vector<NormalContactConstraint>& constraints,
		const Shape& shapePoints, const Shape& shapeOther, const ContactPlane& worldContactPlane)
	{
		auto& tPoints = shapePoints.GetOwner().GetTransform();
		auto p0 = tPoints.ToLocalPosition(worldContactPlane.GetPoint());

		// Make the normal point away from shapePoints
		auto n = tPoints.ToLocalDirection(worldContactPlane.GetNormal()).InDirectionOf(p0);

		for (auto& p : shapePoints.GetCachedPoints())
		{
			if (Vector3::Dot(p - p0, n) >= 0.0f)
			{
				auto pWorld = tPoints.ToWorldPosition(p);
				if (PointIsInsideShape(shapeOther, pWorld) && !PointIsAlreadyAdded(constraints, pWorld))
				{
					constraints.emplace_back(NormalContactConstraint(shapePoints, shapeOther,
						ContactPlane(pWorld, worldContactPlane.GetNormal(), worldContactPlane.GetPeneration())));
				}
			}
		}
	}

	ContactManifold SetUpManifold(const Shape& shape1, const Shape& shape2, std::vector<NormalContactConstraint>& constraints) const
	{
		auto centre = Vector3::Zero();
		auto end = static_cast<int>(constraints.size());

		for (int i = m_StartOfCurrManifold; i < end; i++)
		{
			centre += constraints[i].GetPoint();
		}

		centre /= static_cast<float>(end - m_StartOfCurrManifold);
		
		auto n = constraints[m_StartOfCurrManifold].GetDirection();

		auto dirA = Vector3::OrthogonalDirection(n);
		auto dirB = Vector3::Cross(n, dirA);

		return ContactManifold(shape1, shape2, m_StartOfCurrManifold, end, dirA, dirB, centre);
	}

	Vector3 ProjectOntoContactPlane(const Vector3& p)
	{

	}

	bool PointIsAlreadyAdded(const std::vector<NormalContactConstraint>& constraints, const Vector3& p)
	{
		static constexpr auto equalTol = 0.0001f;
		auto pOnPlane = ProjectOntoContactPlane(p);

		for (int i = m_StartOfCurrManifold; i < constraints.size(); i++)
		{
			auto x = ProjectOntoContactPlane(constraints[i].GetPoint());
			if ((x - pOnPlane).MagnitudeSqr() < equalTol)
				return true;
		}
		return false;
	}

public:
	void Find(std::vector<NormalContactConstraint>& constraints, std::vector<ContactManifold>& manifolds, const Shape& shape1, const Shape& shape2, const ContactPlane& contactPlane)
	{
		m_StartOfCurrManifold = constraints.size();
		m_ContactPlane = contactPlane;

		FindConstraints(constraints, shape1, shape2, contactPlane);
		FindConstraints(constraints, shape2, shape1, contactPlane);

		if (!PointIsAlreadyAdded(constraints, contactPlane.GetPoint()))
			constraints.emplace_back(NormalContactConstraint(shape1, shape2, contactPlane));

		manifolds.emplace_back(SetUpManifold(shape1, shape2, constraints));
	}

private:
	int m_StartOfCurrManifold;
	ContactPlane m_ContactPlane;
};