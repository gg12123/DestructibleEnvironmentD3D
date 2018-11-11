// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include "ShapePoint.h"
#include "ObjectWithHash.h"
#include <assert.h>
#include <vector>

class Face;
class SplitShapeEdge;

/**
 * 
 */
class ShapeEdge : public ObjectWithHash<ShapeEdge>
{
public:
	ShapeEdge();

	void RegisterFace(Face& f, int index)
	{

	}

	const ShapePoint& GetP0() const
	{
		return *m_P0;
	}

	const ShapePoint& GetP1() const
	{
		return *m_P1;
	}

	ShapePoint& GetStart(const Face& requester) const
	{
		// use index in face
	}

	ShapePoint& GetEnd(const Face& requester) const
	{
		// use index in face
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

	Face& GetFace1() const
	{
		return *m_Face1;
	}

	Face& GetFace2() const
	{
		return *m_Face1;
	}

	Face& GetOther(const Face& f) const
	{

	}

	void SetBeenCollected(bool val)
	{
		m_BeenCollectedByShape = val;
	}

	bool HasBeenCollected() const
	{
		return m_BeenCollectedByShape;
	}

private:
	Face* m_Face1 = nullptr;
	Face* m_Face2 = nullptr;

	int m_IndexInFace1;
	int m_IndexInFace2;

	const ShapePoint* m_P0;
	const ShapePoint* m_P1;

	bool m_BeenCollectedByShape = false;
};
