#pragma once
#include "Face.h"
#include "Vector3.h"
#include "Vector2.h"

class FaceOrderingComparator
{
private:
	bool InfinateLineIntersectsFace(const Face& f, const Vector3& lineP0, const Vector3& lineP1)
	{
		Vector2 x;

		auto p0Face = f.ToFaceSpacePosition(lineP0);
		auto p1Face = f.ToFaceSpacePosition(lineP1);

		auto& polyPoints = f.GetFacePoly().GetPoints();

		for (auto i = 0u; i < polyPoints.size(); i++)
		{
			auto nextI = (i + 1u) % polyPoints.size();

			if (Vector2::InfinateLineIntersectsLine(p0Face, p1Face, polyPoints[i], polyPoints[nextI], x))
				return true;
		}
		return false;
	}

	void FindFacePlane(const Face& faceA, const Face& faceB)
	{
		Vector3 p0, p1, dir;
		if (Vector3::FindLineDefinedByTwoPlanes(faceA.GetPlaneP0(), faceA.GetNormal(), faceB.GetPlaneP0(), faceB.GetNormal(), p0, dir))
		{
			p1 = p0 + dir;

			if (InfinateLineIntersectsFace(faceA, p0, p1))
			{
				m_FacePlane = &faceA;
				m_FaceOther = &faceB;
				return;
			}

			// So now either the line intersects face B and not face A, or the line
			// intersects neither face. Either way, face B is an ok choice for the
			// plane face.
		}

		m_FacePlane = &faceB;
		m_FaceOther = &faceA;
	}

	bool OtherFaceIsBeforeFacePlane(const Vector3& rayDir)
	{
		auto n = m_FacePlane->GetNormal().InDirectionOf(-rayDir);
		auto p0 = m_FacePlane->GetPlaneP0();

		auto c = m_FaceOther->GetCentre();

		return Vector3::Dot(c - p0, n) > 0.0f;
	}

public:
	bool FaceAIsBeforeFaceB(const Face& faceA, const Face& faceB, const Vector3& rayDir)
	{
		FindFacePlane(faceA, faceB);

		if (&faceA == m_FaceOther)
			return OtherFaceIsBeforeFacePlane(rayDir);
		else
			return !OtherFaceIsBeforeFacePlane(rayDir);
	}

private:
	const Face* m_FacePlane = nullptr;
	const Face* m_FaceOther = nullptr;
};
