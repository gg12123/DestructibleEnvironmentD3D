#pragma once
#include <vector>
#include "Face.h"
#include "ShapePoint.h"
#include "Vector3.h"
#include "Vector2.h"
#include "UpdatableBound.h"

enum class FaceOrder
{
	FaceAOverlaysB,
	FaceBOverlaysA,
	NoOverlay,
	Unkown // This is when there is no seperating plane
};

class PotentialSeperatingPlane
{
private:
	bool PointIsInsidePlane(const ShapePoint& p)
	{
		return (&p == m_P0) || (&p == m_P1) || (&p == m_P2);
	}

	bool CalculatePlane()
	{
		m_PlaneP0 = m_P0->GetPoint();

		auto u = m_P1->GetPoint() - m_PlaneP0;
		auto v = m_P2->GetPoint() - m_PlaneP0;

		m_PlaneN = Vector3::Cross(u, v);

		auto mag = m_PlaneN.Magnitude();
		if (mag > 0.0f)
		{
			m_PlaneN /= mag;
			return true;
		}
		return false;
	}

	UpdatableBound<float> CalculateFacesComps(const Face& face)
	{
		auto& pointObjs = face.GetPointObjects();
		auto& points = face.GetCachedPoints();

		auto count = points.size();

		UpdatableBound<float> b;

		for (auto i = 0u; i < count; i++)
		{
			auto comp = PointIsInsidePlane(*pointObjs[i]) ?
				0.0f :
				Vector3::Dot(m_PlaneN, points[i] - m_PlaneP0);

			b.Update(comp);
		}
		return b;
	}

public:
	void SetFaces(const Face& faceA, const Face& faceB)
	{
		m_FaceA = &faceA;
		m_FaceB = &faceB;
	}

	bool SeparatesFaces(const ShapePoint& p0, const ShapePoint& p1, const ShapePoint& p2)
	{
		m_P0 = &p0;
		m_P1 = &p1;
		m_P2 = &p2;

		if (!CalculatePlane())
			return false;

		auto faceAComps = CalculateFacesComps(*m_FaceA);
		auto faceBComps = CalculateFacesComps(*m_FaceB);

		return !faceAComps.Overlaps(faceBComps);
	}

	const auto& GetNormal() const
	{
		return m_PlaneN;
	}

	const auto& GetP0() const
	{
		return m_PlaneP0;
	}

private:
	const ShapePoint* m_P0 = nullptr;
	const ShapePoint* m_P1 = nullptr;
	const ShapePoint* m_P2 = nullptr;

	Vector3 m_PlaneN;
	Vector3 m_PlaneP0;

	const Face* m_FaceA = nullptr;
	const Face* m_FaceB = nullptr;
};

class FaceOrderingComparatorState
{
public:
	void Reset()
	{
		m_BOverlaysADetected = false;
		m_AOverlaysBDetected = false;
	}

	void Update(FaceOrder newlyDetectedOrder)
	{
		if (newlyDetectedOrder == FaceOrder::FaceAOverlaysB)
			m_AOverlaysBDetected = true;

		if (newlyDetectedOrder == FaceOrder::FaceBOverlaysA)
			m_BOverlaysADetected = true;
	}

	FaceOrder GetOrder()
	{
		if (m_AOverlaysBDetected && m_BOverlaysADetected)
			return FaceOrder::NoOverlay;

		if (m_AOverlaysBDetected)
			return FaceOrder::FaceAOverlaysB;

		if (m_BOverlaysADetected)
			return FaceOrder::FaceBOverlaysA;

		return FaceOrder::Unkown;
	}

private:
	bool m_AOverlaysBDetected;
	bool m_BOverlaysADetected;
};

class FaceOrderingComparator
{
private:
	FaceOrder ImplicationOfCurrPlane()
	{
		auto n = m_Plane.GetNormal().InDirectionOf(m_RayInAToBDir);

		return Vector3::Dot(n, m_BCentre - m_Plane.GetP0()) > 0.0f ?
			FaceOrder::FaceAOverlaysB :
			FaceOrder::FaceBOverlaysA;
	}

	FaceOrder CalculateImplication(const ShapePoint& p0, const ShapePoint& p1, const ShapePoint& p2)
	{
		if (m_Plane.SeparatesFaces(p0, p1, p2))
			return ImplicationOfCurrPlane();

		return FaceOrder::Unkown;
	}

	void UpdateState(const std::vector<ShapePoint*>& points, const std::vector<ShapePoint*>& pointsForEdges)
	{
		for (auto i = 0u; i < points.size(); i++)
		{
			auto& p0 = *points[i];

			for (auto j = 0u; j < pointsForEdges.size(); j++)
			{
				auto& p1 = *pointsForEdges[j];
				auto& p2 = *pointsForEdges[(j + 1u) % pointsForEdges.size()];

				m_State.Update(CalculateImplication(p0, p1, p2));
			}
		}
	}

public:
	FaceOrder CompareFaces(const Face& faceA, const Face& faceB, const Vector3& rayDir)
	{
		// Face A overlays face B if all seperating planes imply face A is hit first by the ray.
		// Face B overlays face A if all seperating planes imply face B is hit first by the ray.
		// If not all planes imply the same result, the 2 faces do not overlay.

		m_Plane.SetFaces(faceA, faceB);

		m_ACentre = faceA.GetCentre();
		m_BCentre = faceB.GetCentre();
		m_RayInAToBDir = rayDir.InDirectionOf(m_BCentre - m_ACentre);

		m_State.Reset();

		auto& aPoints = faceA.GetPointObjects();
		auto& bPoints = faceB.GetPointObjects();

		m_State.Update(CalculateImplication(*aPoints[0], *aPoints[1], *aPoints[2]));
		m_State.Update(CalculateImplication(*bPoints[0], *bPoints[1], *bPoints[2]));

		UpdateState(aPoints, bPoints);
		UpdateState(bPoints, aPoints);

		return m_State.GetOrder();
	}

private:
	PotentialSeperatingPlane m_Plane;
	FaceOrderingComparatorState m_State;

	Vector3 m_RayInAToBDir;
	Vector3 m_ACentre;
	Vector3 m_BCentre;
};
