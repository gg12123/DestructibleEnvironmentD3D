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
	ShapeEdge(const ShapePoint& p0, const ShapePoint& p1, const Vector3& dirFromP0ToP1)
	{
		m_P0 = &p0;
		m_P1 = &p1;
		m_DirFromP0ToP1 = dirFromP0ToP1;
	}

	void RegisterFace(Face& f, int indexOfThisEdgeInTheFace)
	{
		if (!m_Face1)
		{
			m_Face1 = &f;
			m_IndexInFace1 = indexOfThisEdgeInTheFace;
			return;
		}

		if (!m_Face2)
		{
			m_Face2 = &f;
			m_IndexInFace2 = indexOfThisEdgeInTheFace;
			return;
		}

		assert(false);
	}

	void DeRegisterFace(const Face& f)
	{
		if (&f == m_Face1)
		{
			m_Face1 = nullptr;
			return;
		}

		if (&f == m_Face2)
		{
			m_Face2 = nullptr;
			return;
		}

		assert(false);
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
		return *requester.GetPointObjects()[GetIndex(requester)];
	}

	ShapePoint& GetEnd(const Face& requester) const
	{
		return *requester.GetPointObjects()[requester.NextPointIndex(GetIndex(requester))];
	}

	int GetIndex(const Face& f) const
	{
		if (&f == m_Face1)
			return m_IndexInFace1;

		if (&f == m_Face2)
			return m_IndexInFace2;

		assert(false);
	}

	Vector3 GetDirection(const Face& requester) const
	{
		return requester.GetEdgeDirections()[GetIndex(requester)];
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
		if (&f == m_Face1)
			return *m_Face2;

		if (&f == m_Face2)
			return *m_Face1;

		assert(false);
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

	const ShapePoint* GetConnection(const ShapeEdge& other) const
	{
		if (m_P0 == other.m_P0 || m_P0 == other.m_P1)
			return m_P0;

		if (m_P1 == other.m_P0 || m_P1 == other.m_P1)
			return m_P1;

		return nullptr;
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
