// Fill out your copyright notice in the Description page of Project Settings.
#include "pch.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "Shape.h"
#include "ShapePoint.h"

bool Face::PointIsOnFace(const Vector3& pointShapesSpace) const
{
	return m_FacePoly.PointIsInsideConvexMethod(ToFaceSpacePosition(pointShapesSpace));
}

void  Face::AddPoint(ShapePoint& point, ShapeEdge& edgeToNext)
{
	m_PointObjects.emplace_back(&point);
	m_EdgeObjects.emplace_back(&edgeToNext);

	edgeToNext.RegisterFace(*this, m_EdgeObjects.size() - 1);
}

void Face::ReplaceEdge(const ShapeEdge& existing, ShapeEdge& newEdge)
{
	for (auto i = 0u; i < m_EdgeObjects.size(); i++)
	{
		if (m_EdgeObjects[i] == &existing)
		{
			m_EdgeObjects[i] = &newEdge;
			newEdge.RegisterFace(*this, i);
			return;
		}
	}
	assert(false);
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

		po.OnSplittingFinished(owner);
		eo.OnSplittingFinished(owner);
	}
	ResetHash();
}

Vector3 Face::GetEdgeNormal(const ShapeEdge& edge) const
{
	return GetEdgeNormal(edge.GetIndex(*this));
}

Vector3 Face::GetEdgeNormal(int index) const
{
	return ToShapeSpaceDirection(m_FacePoly.GetNormalAt(index));
}

Vector3 Face::GetEdgeDirection(const ShapeEdge& edge) const
{
	return GetEdgeDirection(edge.GetIndex(*this));
}

Vector3 Face::GetEdgeDirection(int index) const
{
	return (m_CachedPoints[NextPointIndex(index)] - m_CachedPoints[index]).Normalized();
}

void Face::ReplacePointObjects(const ShapePoint& oldP0, const ShapePoint& oldP1, ShapePoint& replacement)
{
	for (auto it = m_PointObjects.begin(); it != m_PointObjects.end(); it++)
	{
		auto p = *it;

		if (&oldP0 == p || &oldP1 == p)
			*it = &replacement;
	}
}

FaceEdgeCaseResult Face::CastToEdgeInside(const Vector3& origin, const Vector3& dir) const
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