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
	ShapeEdge(ShapePoint& p0, ShapePoint& p1, const Vector3& dirFromP0ToP1)
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

	ShapePoint& GetP0() const
	{
		return *m_P0;
	}

	ShapePoint& GetP1() const
	{
		return *m_P1;
	}

	ShapePoint& GetStart(const Face& requester) const;
	ShapePoint& GetEnd(const Face& requester) const;

	int GetIndex(const Face& f) const
	{
		if (&f == m_Face1)
			return m_IndexInFace1;

		if (&f == m_Face2)
			return m_IndexInFace2;

		assert(false);
		return -1;
	}

	Vector3 GetDirection(const Face& requester) const;

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
		return *m_Face1;
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

	void ReplacePoint(const ShapePoint& existing, ShapePoint& newPoint)
	{
		if (&existing == m_P0)
			m_P0 = &newPoint;

		if (&existing == m_P1)
			m_P1 = &newPoint;

		assert(false);
	}

	bool IsAttachedTo(const Face& f)
	{
		return (&f == m_Face1) || (&f == m_Face2);
	}

private:
	Face* m_Face1 = nullptr;
	Face* m_Face2 = nullptr;

	int m_IndexInFace1;
	int m_IndexInFace2;

	ShapePoint* m_P0 = nullptr;
	ShapePoint* m_P1 = nullptr;

	SplitShapeEdge* m_SplitEdge = nullptr;

	bool m_BeenCollectedByShape = false;

	Vector3 m_DirFromP0ToP1;
};
