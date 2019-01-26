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
	class ClosestPlanes
	{
	private:
		bool AlreadyGotPlane(int id, int& index) const
		{
			for (auto i = 0; i < MaxNumPlanes; i++)
			{
				if (m_PlanesIds[i] == id)
				{
					index = i;
					return true;
				}
			}
			return false;
		}

	public:
		static constexpr int MaxNumPlanes = 3;

		void Reset()
		{
			for (int i = 0; i < MaxNumPlanes; i++)
			{
				m_Distances[i] = MathU::Infinity;
				m_PlanesIds[i] = -1;
			}
			m_WithinTolPlanes.clear();
		}

		void Update(int id, const Plane& plane, float dist)
		{
			int indexOfExisting;
			if (AlreadyGotPlane(id, indexOfExisting))
			{
				if (m_Distances[indexOfExisting] < dist)
				{
					m_Distances[indexOfExisting] = dist;
					m_Planes[indexOfExisting] = plane;
				}
				return;
			}

			for (int i = 0; i < MaxNumPlanes; i++)
			{
				if (m_Distances[i] < dist)
				{
					m_Distances[i] = dist;
					m_Planes[i] = plane;
					m_PlanesIds[i] = id;
					return;
				}
			}
		}

		void FinishUpdating(float tol)
		{
			Plane closest;
			auto closestDist = MathU::Infinity;

			for (int i = 0; i < MaxNumPlanes; i++)
			{
				auto dist = m_Distances[i];

				if (dist <= tol)
					m_WithinTolPlanes.emplace_back(m_Planes[i]);

				if (dist < closestDist)
				{
					closestDist = dist;
					closest = m_Planes[i];
				}
			}

			if (m_WithinTolPlanes.size() == 0u)
				m_WithinTolPlanes.emplace_back(closest);
		}

		const auto& GetNearbyPlanes() const
		{
			return m_WithinTolPlanes;
		}

	private:
		std::array<int, MaxNumPlanes> m_PlanesIds;
		std::array<Plane, MaxNumPlanes> m_Planes;
		std::array<float, MaxNumPlanes> m_Distances;

		std::vector<Plane> m_WithinTolPlanes;
	};

	float DistanceFromPointToFace(const Vector3& p, const Face& f) const
	{

	}

	void FindNearbyPlanes(const std::vector<Face*>& faces, const Vector3& p)
	{
		m_NearbyPlanes.Reset();

		for (auto f : faces)
		{
			auto dist = DistanceFromPointToFace(p, *f);
			m_NearbyPlanes.Update(f->GetPlaneId(), f->ToPlane(), dist);
		}
		m_NearbyPlanes.FinishUpdating(0.1f);
	}

	Plane CalculateSplitPlane(const Vector3& p) const
	{
		auto& nearbyPlanes = m_NearbyPlanes.GetNearbyPlanes();

		switch (nearbyPlanes.size())
		{
		case 1:
		{
			return nearbyPlanes[0];
		}
		case 2:
		{
			Ray inter;
			if (!nearbyPlanes[0].Intersects(nearbyPlanes[1], inter))
				return nearbyPlanes[0];

			auto p0 = inter.ClosestPointOnRay(p);
			auto n = (nearbyPlanes[0].GetNormal() + nearbyPlanes[1].GetNormal()).Normalized();

			return Plane(n, p0);
		}
		case 3:
		{
			Vector3 inter;
			if (!nearbyPlanes[0].Intersects(nearbyPlanes[1], nearbyPlanes[2], inter))
				return nearbyPlanes[0];

			return Plane((nearbyPlanes[0].GetNormal() + nearbyPlanes[1].GetNormal() + nearbyPlanes[2].GetNormal()).Normalized(),
				inter);
		}
		default:
		{
			assert(false);
			return Plane();
		}
		}
	}

	Matrix4 CalculateCutShapesTransform(Transform& toSplitsTransform, const Plane& splitPlane) const
	{
		auto v = (Vector3(1.0f, 1.0f, 1.0f)).Normalized();
		auto u = Vector3::OrthogonalDirection(v);
		auto k = Vector3::Cross(v, u);

		Matrix4 R0;
		R0.SetColumn(0, u.x, k.x, v.x, 0.0f);
		R0.SetColumn(1, u.y, k.y, v.y, 0.0f);
		R0.SetColumn(2, u.z, k.z, v.z, 0.0f);
		R0.SetColumn(3, 0.0f, 0.0f, 0.0f, 1.0f);

		auto scaleFactor = 10.0f;
		auto S = Matrix4::FromScale(scaleFactor, scaleFactor, scaleFactor);

		auto& n = splitPlane.GetNormal();
		auto R1 = Matrix4::FromRotation(Quaternion::LookRotation(-n, Vector3::OrthogonalDirection(n))); // could have some randomnes for up
		
		auto& P = splitPlane.GetP0();
		auto D = 0.25f * P.Magnitude(); // could have some randomnes

		auto sV = (Vector3(scaleFactor, scaleFactor, scaleFactor)).Magnitude();
		auto T = Matrix4::FromTranslation(P + (sV - D) * n);

		return T * R1 * R0 * S;
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

		auto M = CalculateCutShapesTransform(toSplitsTransform, splitPlane);

		// Add faces

		m_CutShape.OnAllFacesAdded();

		return m_CutShape;
	}

private:
	Shape m_CutShape;

	ClosestPlanes m_NearbyPlanes;
};