#pragma once
#include <vector>
#include "CollisionData.h"
#include "CompoundShape.h"
#include "Shape.h"
#include "SatOptimisedCollisionDetection.h"
#include "Debug.h"
#include "DynamicTriangleArray.h"
#include "ContactPointFinder.h"
#include "GjkCollisionDetection.h"
#include "EpaContact.h"
#include "ContactContexts.h"

class CollisionDetector
{
private:
	void TransformPointsToShape1sSpace(const Shape& shape2)
	{
		m_TransformedPoints.clear();

		for (auto& p : shape2.GetCachedPoints())
			m_TransformedPoints.emplace_back(m_ToShape1sSpace * p);
	}

	void TransformNormalsToShape1sSpace(const Shape& shape2)
	{
		m_TransformedNormals.clear();

		for (auto& n : shape2.GetCachedFaceNormals())
			m_TransformedNormals.emplace_back(m_DirToShape1sSpace.RotateV(n));
	}

	void InitTransformMatrices(const CompoundShape& shape1, const CompoundShape& shape2)
	{
		auto& t1 = shape1.GetTransform();
		auto& t2 = shape2.GetTransform();

		m_ToShape1sSpace = t1.GetWorldToLocalMatrix() * t2.GetLocalToWorldMatrix();
		m_DirToShape1sSpace = t1.GetWorldToLocalRotation() * t2.GetLocalToWorldRotation();
		m_ActiveTransform = &t1;
	}

	GjkCollisionDetector::Simplex GetInitialSimplex(const ContactContext& context, const GjkInputShape& shape1, const GjkInputShape& shape2)
	{
		if (context.TestedOnPrevTick)
		{
			auto& s = m_Simplices[context.IndexOfSimplex];
			s.ReCalculatePoints(shape1, shape2);
			return s;
		}
		return GjkCollisionDetector::Simplex();
	}

public:
	void PrepareToFindContacts()
	{
		m_SimplicesNext.swap(m_Simplices);
		m_SimplicesNext.clear();
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& contact, SimdStdVector<Vector3>& contactPoints)
	{
		InitTransformMatrices(shape1.GetOwner(), shape2.GetOwner());

		TransformPointsToShape1sSpace(shape2);

		auto gjkShape1 = GjkInputShape(shape1.GetCachedPoints(), shape1.GetCentre());
		auto gjkShape2 = GjkInputShape(m_TransformedPoints, m_ToShape1sSpace * shape2.GetCentre());

		auto simplex = GetInitialSimplex(context, gjkShape1, gjkShape2);

		auto gjkRes = m_GjkDetector.Run(gjkShape1, gjkShape2, simplex);

		context.IndexOfSimplex = m_SimplicesNext.size();
		m_SimplicesNext.emplace_back(simplex);

		switch (gjkRes)
		{
		case GjkCollisionDetector::GjkResult::Intersection:
		case GjkCollisionDetector::GjkResult::MaybeIntersection:
		{
			ContactPlane localContact;
			auto epaRes = m_EpaContactFinder.FindContact(gjkShape1, gjkShape2, simplex, localContact);

			if (epaRes == EpaContact::EpaResult::NoContact)
			{
				if (gjkRes == GjkCollisionDetector::GjkResult::Intersection)
					Debug::Log(std::string("Epa and gjk results are inconsistent."));

				return false;
			}

			if (epaRes == EpaContact::EpaResult::DegenerateSimplex)
			{
				Debug::Log(std::string("Degenerate simplex fed to Epa!!!!!!"));
				return false;
			}

			TransformNormalsToShape1sSpace(shape2);

			auto satShape1 = SatInputShape(shape1.GetEdgeIndexesPoints(), shape1.GetEdgeIndexsFaces(),
				shape1.GetCachedPoints(), shape1.GetCachedFaceNormals(), shape1.GetFaceP0Indexes(), shape1.GetCentre());

			auto satShape2 = SatInputShape(shape2.GetEdgeIndexesPoints(), shape2.GetEdgeIndexsFaces(),
				m_TransformedPoints, m_TransformedNormals, shape2.GetFaceP0Indexes(), m_ToShape1sSpace * shape2.GetCentre());

			for (auto& p : m_ContactPointFinder.FindContactPoints(satShape1, satShape2, localContact))
				contactPoints.emplace_back(m_ActiveTransform->ToWorldPosition(p));

			contact = ContactPlane(localContact.GetContactMin(), localContact.GetContactMax(),
				m_ActiveTransform->ToWorldDirection(localContact.GetNormal()));

			return true;
		}
		case GjkCollisionDetector::GjkResult::NoIntersection:
		{
			return false;
		}
		default:
			break;
		}
		assert(false);
		return false;
	}

private:
	SimdStdVector<Vector3> m_TransformedPoints;
	SimdStdVector<Vector3> m_TransformedNormals;

	SimdStdVector<GjkCollisionDetector::Simplex> m_Simplices;
	SimdStdVector<GjkCollisionDetector::Simplex> m_SimplicesNext;

	Matrix4 m_ToShape1sSpace;
	Quaternion m_DirToShape1sSpace;
	const Transform* m_ActiveTransform;
	ContactPointFinder m_ContactPointFinder;
	GjkCollisionDetector m_GjkDetector;
	EpaContact m_EpaContactFinder;
};