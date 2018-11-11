// Fill out your copyright notice in the Description page of Project Settings.
#include "pch.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "Shape.h"
#include "ShapePoint.h"
#include "NewPointsGetter.h"

bool Face::PointIsInsideFace(const Vector3& pointShapesSpace) const
{
	return m_FacePoly.PointIsInsideAssumingConvex(ToFaceSpacePosition(pointShapesSpace)) == PointInPolyCase::Inside;
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