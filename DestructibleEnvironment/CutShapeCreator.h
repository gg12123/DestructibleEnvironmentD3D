#pragma once
#include <array>
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"
#include "Matrix.h"
#include "CubeFacesCreator.h"
#include "Random.h"
#include "Plane.h"

class CutShapeCreator
{
private:
	bool FaceIsNearby(const Face& f, const Vector3& p) const
	{
		static const auto nearbyTol = 0.1f;
	}

	void FindNearbyPlanes(const std::vector<Face*>& faces, const Vector3& p)
	{
		m_UsedPlanes.clear();
		m_NearbyPlanes.clear();

		for (auto f : faces)
		{
			// TODO - should also check that the normals dont point in opposite directions.
			if (FaceIsNearby(*f, p) && !CollectionU::Contains(m_UsedPlanes, f->GetPlaneId()))
			{
				m_UsedPlanes.emplace_back(f->GetPlaneId());
				m_NearbyPlanes.emplace_back(f->ToPlane());

				if (m_NearbyPlanes.size() == 3u)
					break;
			}
		}
	}

	Plane CalculateSplitPlane(const Vector3& p) const
	{
		switch (m_NearbyPlanes.size())
		{
		case 1:
		{
			return m_NearbyPlanes[0];
		}
		case 2:
		{
			Ray inter;
			if (!m_NearbyPlanes[0].Intersects(m_NearbyPlanes[1], inter))
				return m_NearbyPlanes[0];

			auto p0 = inter.ClosestPointOnRay(p);
			auto n = (m_NearbyPlanes[0].GetNormal() + m_NearbyPlanes[1].GetNormal()).Normalized();

			return Plane(n, p0);
		}
		case 3:
		{
			Vector3 inter;
			if (!m_NearbyPlanes[0].Intersects(m_NearbyPlanes[1], m_NearbyPlanes[2], inter))
				return m_NearbyPlanes[0];

			return Plane((m_NearbyPlanes[0].GetNormal() + m_NearbyPlanes[1].GetNormal() + m_NearbyPlanes[2].GetNormal()).Normalized(),
				inter);
		}
		default:
		{
			assert(false);
			return Plane();
		}
		}
	}

	Matrix4 CalculateCutShapesTransform(Transform& toSplitsTransform, const Plane& splitPlane, Quaternion& q) const
	{
		
	}

	Matrix4 CalculateCutShapesTransform(Transform& toSplitsTransform, const Vector3& splitPoint, const Vector3& splitNormal, Quaternion& q) const
	{
		auto sZ = 100.0f;
		auto r = 0.0f; // Random::Range(0.0f, 0.5f);

		auto A = r * splitPoint;
		auto C = A + sZ * splitNormal;

		q = Quaternion::LookRotation(-splitNormal, Vector3::OrthogonalDirection(splitNormal));

		return Matrix4::FromTranslation(C) * Matrix4::FromRotation(q) * Matrix4::FromScale(sZ, sZ, sZ);
	}

public:
	// Split point should be on the surface of one of the faces.
	Shape & Create(Shape& toSplit, const Vector3& splitPointWorld, int firstPlaneId)
	{
		auto& toSplitsTransform = toSplit.GetTransform();

		m_CutShape.Clear();
		m_CutShape.GetTransform().SetEqualTo(toSplitsTransform);

		auto splitPoint = toSplitsTransform.ToLocalPosition(splitPointWorld);
		FindNearbyPlanes(toSplit.GetFaces(), splitPoint);
		auto splitPlane = CalculateSplitPlane(splitPoint);

		Quaternion q;
		auto M = CalculateCutShapesTransform(toSplitsTransform, splitPlane, q);

		m_FacesCreator.CreateFaces(m_CutShape, M, q, firstPlaneId);

		m_CutShape.OnAllFacesAdded();

		return m_CutShape;
	}

private:
	Shape m_CutShape;
	CubeFacesCreator m_FacesCreator;

	std::vector<Plane> m_NearbyPlanes;
	std::vector<int> m_UsedPlanes;
};