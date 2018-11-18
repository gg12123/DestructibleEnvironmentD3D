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
	ShapeEdge(const ShapePoint& p0, const ShapePoint& p1)
	{
		m_P0 = &p0;
		m_P1 = &p1;
	}

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
		return *m_SplitEdge;
	}

	void SetSplitEdge(SplitShapeEdge& se)
	{
		m_SplitEdge = &se;
	}

	bool IsSplit() const
	{
		return m_SplitEdge;
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

	const Vector3& GetDirFromP0ToP1() const
	{
		return m_DirFromP0ToP1;
	}

	ShapePoint* GetConnection(const ShapeEdge& other)
	{

	}

private:
	Face* m_Face1 = nullptr;
	Face* m_Face2 = nullptr;

	int m_IndexInFace1;
	int m_IndexInFace2;

	const ShapePoint* m_P0 = nullptr;
	const ShapePoint* m_P1 = nullptr;

	SplitShapeEdge* m_SplitEdge = nullptr;

	bool m_BeenCollectedByShape = false;

	Vector3 m_DirFromP0ToP1; // Must be set at initialisation - not safe to calculate because P0 and P1 can be coincident.
};
