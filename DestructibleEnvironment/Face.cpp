// Fill out your copyright notice in the Description page of Project Settings.
#include "pch.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "Shape.h"
#include "ShapePoint.h"

bool Face::PointIsOnFace(const Vector3& pointShapesSpace) const
{
	return m_FacePoly.PointIsInsideOrOnEdge(ToFaceSpacePosition(pointShapesSpace));
}

void  Face::AddPoint(ShapePoint& point, const Vector3& dirToNext, ShapeEdge& edgeToNext)
{
	m_PointObjects.emplace_back(&point);
	m_EdgeObjects.emplace_back(&edgeToNext);
	m_EdgeDirections.emplace_back(dirToNext);

	edgeToNext.RegisterFace(*this, m_EdgeObjects.size() - 1);
}

void Face::OnSplittingFinished(Shape& owner)
{
	m_OwnerShape = &owner;

	m_CachedPoints.clear();
	m_FacePoly.Clear();
	InitFaceCoOrdinateSystem(m_PointObjects[0]->GetPoint());

	for (auto i = 0U; i < m_PointObjects.size(); i++)
	{
		auto& po = (*m_PointObjects[i]);
		auto& eo = (*m_EdgeObjects[i]);

		auto& p = po.GetPoint();

		m_CachedPoints.emplace_back(p);
		m_FacePoly.Add(ToFaceSpacePosition(p));

		po.ResetHash();
		po.SetBeenCollected(false);

		eo.ResetHash();
		eo.SetBeenCollected(false);
	}
	ResetHash();
}

Vector3 Face::GetEdgeNormal(const ShapeEdge& edge) const
{
	return GetEdgeNormal(edge.GetIndex(*this));
}

Vector3 Face::GetEdgeNormal(int index) const
{
	// Dont use the poly directly becasue the poly may
	// have coincident points.

	auto dirFaceSpace = ToFaceSpaceDirection(m_EdgeDirections[index]);
	
	// TODO - is this the correct rotation direction?
	auto normalFaceSpace = Vector2(-dirFaceSpace.y, dirFaceSpace.x);

	return ToShapeSpaceDirection(normalFaceSpace);
}

FaceEdgeCaseResult Face::CastToEdgeInside(const Vector3& origin, const Vector3& dir)
{
	auto origin2 = ToFaceSpacePosition(origin);
	auto dir2 = ToFaceSpaceDirection(dir);

	auto res = FaceEdgeCaseResult();

	for (auto i = 0U; i < m_FacePoly.GetPointCount(); i++)
	{
		auto nextI = m_FacePoly.NextIndex(i);

		auto& p0 = m_FacePoly.GetPointAt(i);
		auto& p1 = m_FacePoly.GetPointAt(nextI);

		Vector2 intPoint;
		if (Vector2::RayIntersectsLine(origin2, dir2, p0, p1, intPoint) && (Vector3::Dot(dir, GetEdgeNormal(i)) > 0.0f))
		{
			auto dist = (origin2 - intPoint).Magnitude();

			if (dist < res.Distance)
				res = FaceEdgeCaseResult(*m_EdgeObjects[i], ToShapeSpacePosition(intPoint), dist);
		}
	}
	return res;
}