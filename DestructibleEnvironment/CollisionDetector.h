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
#include "PhysicsTypes.h"

class CollisionDetectorConvexMeshConvexMesh
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

	bool FindContactPlaneLocalSpace(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& localContact)
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
			auto epaRes = m_EpaContactFinder.FindContact(gjkShape1, gjkShape2, simplex, localContact);

			if (epaRes == EpaContact::EpaResult::NoContact)
			{
				return false;
			}

			if (epaRes == EpaContact::EpaResult::DegenerateSimplex)
			{
				Debug::Log(std::string("Degenerate simplex fed to Epa!!!!!!"));
				return false;
			}

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

public:
	void PrepareToFindContacts()
	{
		m_SimplicesNext.swap(m_Simplices);
		m_SimplicesNext.clear();
	}

	void SaveSimplexForNextTick(const Shape& shape1, const Shape& shape2, ContactContext& context)
	{
		// The two shapes are asleep and in contact or pretty close so
		// the simplex is saved for next tick. It will be used when they
		// wake up.

		assert(context.TestedOnPrevTick);

		auto& s = m_Simplices[context.IndexOfSimplex];
		context.IndexOfSimplex = m_SimplicesNext.size();
		m_SimplicesNext.emplace_back(s);
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& contact)
	{
		ContactPlane local;
		if (FindContactPlaneLocalSpace(shape1, shape2, context, local))
		{
			contact = ContactPlane(local.GetContactMin(), local.GetContactMax(),
				m_ActiveTransform->ToWorldDirection(local.GetNormal()));

			return true;
		}
		return false;
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context)
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
			case GjkCollisionDetector::GjkResult::NoIntersection:
			{
				return false;
			}
			case GjkCollisionDetector::GjkResult::Intersection:
			{
				return true;
			}
			case GjkCollisionDetector::GjkResult::MaybeIntersection:
			{
				return (m_EpaContactFinder.FindContact(gjkShape1, gjkShape2, simplex) == EpaContact::EpaResult::Contact);
			}
		}

		assert(false);
		return false;
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& contact, SimdStdVector<Vector3>& contactPoints)
	{
		ContactPlane localContact;
		if (FindContactPlaneLocalSpace(shape1, shape2, context, localContact))
		{
			contact = ContactPlane(localContact.GetContactMin(), localContact.GetContactMax(),
				m_ActiveTransform->ToWorldDirection(localContact.GetNormal()));

			TransformNormalsToShape1sSpace(shape2);

			auto satShape1 = SatInputShape(shape1.GetEdgeIndexesPoints(), shape1.GetEdgeIndexsFaces(),
				shape1.GetCachedPoints(), shape1.GetCachedFaceNormals(), shape1.GetFaceP0Indexes(), shape1.GetCentre());

			auto satShape2 = SatInputShape(shape2.GetEdgeIndexesPoints(), shape2.GetEdgeIndexsFaces(),
				m_TransformedPoints, m_TransformedNormals, shape2.GetFaceP0Indexes(), m_ToShape1sSpace * shape2.GetCentre());

			for (auto& p : m_ContactPointFinder.FindContactPoints(satShape1, satShape2, localContact))
				contactPoints.emplace_back(m_ActiveTransform->ToWorldPosition(p));

			return true;
		}
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

class CollisionDetectorSphereSphere
{
public:
	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& contact, SimdStdVector<Vector3>& contactPoints)
	{
		
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& contact)
	{
		
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context)
	{
		
	}
};

class CollisionDetectorSphereConvexMesh
{
public:
	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& contact, SimdStdVector<Vector3>& contactPoints)
	{
		
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& contact)
	{
		
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context)
	{
		
	}
};

class CollisionDetector
{
private:
	PhysicsGeometryComparisonType GetComparisionType(const Shape& shape1, const Shape& shape2)
	{

	}

	template<class ...Ts>
	bool FindContactLocal(const Shape& shape1, const Shape& shape2, Ts&... args)
	{
		switch (GetComparisionType(shape1, shape2))
		{
		case PhysicsGeometryComparisonType::ConvexMeshConvexMesh:
			return m_ConvexMeshConvexMesh.FindContact(shape1, shape2, std::forward<Ts...>(args...));
		case PhysicsGeometryComparisonType::SphereSphere:
			return m_SphereSphereDetector.FindContact(shape1, shape2, std::forward<Ts...>(args...));
		case PhysicsGeometryComparisonType::SphereConvexMesh:
			return m_SphereConvexMeshDetector.FindContact(shape1, shape2, std::forward<T...>(args...));
		case PhysicsGeometryComparisonType::ConvexMeshSphere:
			return m_SphereConvexMeshDetector.FindContact(shape2, shape1, std::forward<T...>(args...));
		default:
			break;
		}

		assert(false);
		return false;
	}

public:
	void PrepareToFindContacts()
	{
		m_ConvexMeshConvexMesh.PrepareToFindContacts();
	}

	void SaveSimplexForNextTick(const Shape& shape1, const Shape& shape2, ContactContext& context)
	{
		if (GetComparisionType(shape1, shape2) == PhysicsGeometryComparisonType::ConvexMeshConvexMesh)
			m_ConvexMeshConvexMesh.SaveSimplexForNextTick(shape1, shape2, context);
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& contact, SimdStdVector<Vector3>& contactPoints)
	{
		return FindContactLocal(shape1, shape2, context, contact, contactPoints);
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context, ContactPlane& contact)
	{
		return FindContactLocal(shape1, shape2, context, contact);
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactContext& context)
	{
		return FindContactLocal(shape1, shape2, context);
	}

private:
	CollisionDetectorConvexMeshConvexMesh m_ConvexMeshConvexMesh;
	CollisionDetectorSphereSphere m_SphereSphereDetector;
	CollisionDetectorSphereConvexMesh m_SphereConvexMeshDetector;
};