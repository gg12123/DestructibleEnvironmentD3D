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

void Face::InitFaceCoOrdinateSystem()
{
	InitFaceCoOrdinateSystem(m_PointObjects[0]->GetPoint());
}

void Face::OnSplittingFinished(Shape& owner)
{
	CalculateNormalFromPoints();

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

void Face::CalculateNormalFromPoints()
{
	auto& p0 = m_PointObjects[0]->GetPoint();
	auto& p1 = m_PointObjects[1]->GetPoint();
	auto& p2 = m_PointObjects[2]->GetPoint();

	m_Normal = Vector3::Cross(p1 - p0, p2 - p1).Normalized();
}

void Face::MergeWith(const Face& other, const ShapeEdge& commonEdge)
{
	std::vector<ShapePoint*> newPoints;
	std::vector<ShapeEdge*> newEdges;

	auto c = m_PointObjects.size() + other.m_PointObjects.size();
	newPoints.reserve(c);
	newEdges.reserve(c);

	auto& faceA = *this;
	auto& faceB = other;

	auto iA = commonEdge.GetIndex(faceA);
	auto iB = commonEdge.GetIndex(faceB);

	for (auto i = faceA.NextPointIndex(iA); i != iA; i = faceA.NextPointIndex(i))
	{
		newPoints.emplace_back(faceA.m_PointObjects[i]);
		newEdges.emplace_back(faceA.m_EdgeObjects[i]);
	}

	for (auto i = faceB.NextPointIndex(iB); i != iB; i = faceB.NextPointIndex(i))
	{
		newPoints.emplace_back(faceB.m_PointObjects[i]);
		newEdges.emplace_back(faceB.m_EdgeObjects[i]);
	}

	for (auto e : other.GetEdgeObjects())
		e->DeRegisterFace(other);

	for (auto e : m_EdgeObjects)
		e->DeRegisterFace(*this);

	m_PointObjects = std::move(newPoints);
	m_EdgeObjects = std::move(newEdges);

	for (auto i = 0u; i < m_EdgeObjects.size(); i++)
		m_EdgeObjects[i]->RegisterFace(*this, i);
}

void Face::RemovePoint(const ShapePoint& toRemove, ShapeEdge& newEdge)
{
	for (auto e : m_EdgeObjects)
		e->DeRegisterFace(*this);

	auto itEdge = m_EdgeObjects.begin();
	auto index = 0;
	for (auto it = m_PointObjects.begin(); it != m_PointObjects.end(); it++, itEdge++, index++)
	{
		if (*it == &toRemove)
		{
			m_PointObjects.erase(it);
			m_EdgeObjects.erase(itEdge);
			break;
		}
	}

	m_EdgeObjects[CollectionU::GetPrevIndex(m_EdgeObjects, index)] = &newEdge;

	for (auto i = 0u; i < m_EdgeObjects.size(); i++)
		m_EdgeObjects[i]->RegisterFace(*this, i);
}