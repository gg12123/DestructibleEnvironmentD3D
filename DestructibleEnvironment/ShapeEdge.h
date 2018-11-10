// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include "ShapePoint.h"
#include <assert.h>
#include <vector>

class Face;
class SplitShapeEdge;

/**
 * 
 */
class ShapeEdge
{
public:
	ShapeEdge();

	void RegisterFace(const Face& f, int index)
	{

	}

	ShapePoint& GetStart(const Face& requester) const
	{

	}

	ShapePoint& GetEnd(const Face& requester) const
	{

	}

	int GetIndex(const Face& requester) const
	{

	}

	Vector3 GetDirection(const Face& requester) const
	{

	}

	SplitShapeEdge& GetSplitEdge() const
	{

	}

	bool IsSplit() const
	{

	}

private:
	const Face* m_Face1 = nullptr;
	const Face* m_Face2 = nullptr;

	int m_IndexInFace1;
	int m_IndexInFace2;

	const ShapePoint* m_P0;
	const ShapePoint* m_P1;
};
