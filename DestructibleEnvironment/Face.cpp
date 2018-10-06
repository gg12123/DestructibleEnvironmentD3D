// Fill out your copyright notice in the Description page of Project Settings.
#include "pch.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "Shape.h"
#include "Point.h"
#include "NewPointsGetter.h"

bool Face::PointIsInsideFace(const Vector3& pointShapesSpace) const
{
	return m_FacePoly.PointIsInsideAssumingConvex(ToFaceSpacePosition(pointShapesSpace));
}

void Face::RegisterIntersection(const FaceFaceIntersection<Vector3>& inter)
{
	auto& t = m_OwnerShape->GetTransform();

	auto& faceEdge1 = inter.Intersection1;
	auto& faceEdge2 = inter.Intersection2;

	auto p1 = ToFaceSpacePosition(t.ToLocalPosition(faceEdge1.Position));
	auto p2 = ToFaceSpacePosition(t.ToLocalPosition(faceEdge2.Position));

	m_Intersections.emplace_back(FaceFaceIntersection<Vector2>(*inter.Face1, *inter.Face2,
		FaceEdgeIntersection<Vector2>(*faceEdge1.PiercedFace, faceEdge1.PiercingEdge, p1),
		FaceEdgeIntersection<Vector2>(*faceEdge2.PiercedFace, faceEdge2.PiercingEdge, p2)));
}

void Face::ReCentre(const Vector3& centre, Shape& owner)
{
	m_OwnerShape = &owner;
	m_ToSharedPoints.clear();
	m_FacePoly.Clear();

	m_CachedPoints[0] -= centre;

	InitFaceCoOrdinateSystem(m_CachedPoints[0]);
	m_FacePoly.Add(Vector2::Zero());
	m_ToSharedPoints.emplace_back(owner.RegisterPoint(m_CachedPoints[0]));

	for (auto i = 1U; i < m_CachedPoints.size(); i++)
	{
		m_CachedPoints[i] -= centre;
		m_FacePoly.Add(ToFaceSpacePosition(m_CachedPoints[i]));
		m_ToSharedPoints.emplace_back(owner.RegisterPoint(m_CachedPoints[i]));
	}
}
