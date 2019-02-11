// Fill out your copyright notice in the Description page of Project Settings.
#include "pch.h"
#include "ShapeEdge.h"
#include "ShapePoint.h"
#include "Shape.h"
#include "Face.h"

bool ShapeEdge::BridgesCoPlanarFaces() const
{
	assert(false);
	return false;
}

ShapePoint& ShapeEdge::GetStart(const Face& requester) const
{
	return *requester.GetPointObjects()[GetIndex(requester)];
}

ShapePoint& ShapeEdge::GetEnd(const Face& requester) const
{
	return *requester.GetPointObjects()[requester.NextPointIndex(GetIndex(requester))];
}

Vector3 ShapeEdge::GetDirection(const Face& requester) const
{
	return requester.GetEdgeDirection(GetIndex(requester));
}