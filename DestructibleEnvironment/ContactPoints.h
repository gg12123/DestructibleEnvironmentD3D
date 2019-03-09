#pragma once
#include "PhysicsObject.h"
#include "Shape.h"
#include "CollisionData.h"
#include "CollisionResponder.h"
#include "Face.h"

class ContactConstraint
{
public:
	ContactConstraint(const Shape& s1, const Shape& s2, const ContactPlane& contactPlane) :
		m_Body1(s1.GetOwner().ToPhysicsObject()),
		m_Body2(s2.GetOwner().ToPhysicsObject()),
		m_AccumulatedImpulse(0.0f)
	{
		auto refN = m_Body2->GetTransform().ToWorldPosition(s2.GetCentre()) - m_Body1->GetTransform().ToWorldPosition(s1.GetCentre());

		m_ContactPlane = ContactPlane(contactPlane.GetPoint(),
			contactPlane.GetNormal().InDirectionOf(refN),
			contactPlane.GetPeneration());
	}

	void ApplyImpulse(float J) const
	{
		m_Body1->ApplyImpulse(Impulse(-J * m_ContactPlane.GetNormal(), m_ContactPlane.GetPoint(), J));
		m_Body2->ApplyImpulse(Impulse(J * m_ContactPlane.GetNormal(), m_ContactPlane.GetPoint(), J));
	}

	float GetAccumulatedImpulse() const
	{
		return m_AccumulatedImpulse;
	}

	void SetAccumulatedImpulse(float val)
	{
		m_AccumulatedImpulse = val;
	}

	float CalculateImpulse() const
	{
		return CollisionResponder::CalculateImpulse(m_ContactPlane, *m_Body1, *m_Body2);
	}

private:
	PhysicsObject * m_Body1;
	PhysicsObject * m_Body2;
	ContactPlane m_ContactPlane;
	float m_Penetration;
	float m_AccumulatedImpulse;
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

	void FindConstraints(std::vector<ContactConstraint>& constraints,
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
				if (PointIsInsideShape(shapeOther, pWorld))
				{
					constraints.emplace_back(ContactConstraint(shapePoints, shapeOther,
						ContactPlane(pWorld, worldContactPlane.GetNormal(), worldContactPlane.GetPeneration())));
				}
			}
		}
	}

public:
	void Find(std::vector<ContactConstraint>& constraints, const Shape& shape1, const Shape& shape2, const ContactPlane& contactPlane)
	{
		FindConstraints(constraints, shape1, shape2, contactPlane);
		FindConstraints(constraints, shape2, shape1, contactPlane);

		// TODO - only add this one if it is sufficiently different from the others.
		constraints.emplace_back(ContactConstraint(shape1, shape2, contactPlane));
	}
};