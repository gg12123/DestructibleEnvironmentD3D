// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include "ShapePoint.h"
#include "ObjectWithHash.h"
#include <assert.h>
#include <vector>

class Face;
class SplitShapeEdge;
class Shape;

/**
 * 
 */
class ShapeEdge : public ObjectWithHash<ShapeEdge>, public AlignedObject16
{
public:
	ShapeEdge()
	{
	}

	void OnTakenFromPool(ShapePoint& p0, ShapePoint& p1)
	{
		m_P0 = &p0;
		m_P1 = &p1;
		m_OwnerShape = nullptr;
		m_BeenCollectedByShape = false;
		m_SplitEdge = nullptr;
		m_Face1 = nullptr;
		m_Face2 = nullptr;
		ResetHash();
	}

	void OnReturnedToPool() {}

	bool BridgesCoPlanarFaces() const;

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
		return *m_Face2;
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

	ShapePoint& GetOther(const ShapePoint& p) const
	{
		if (&p == m_P0)
			return *m_P1;

		if (&p == m_P1)
			return *m_P0;

		assert(false);
		return *m_P0;
	}

	void OnSplittingFinished(const Shape& owner)
	{
		m_OwnerShape = &owner;
		m_BeenCollectedByShape = false;
		ResetHash();
	}

	const auto& GetOwnerShape() const
	{
		return *m_OwnerShape;
	}

	bool HasBeenCollected() const
	{
		return m_BeenCollectedByShape;
	}

	void SetBeenCollected()
	{
		m_BeenCollectedByShape = true;
	}

	void ClearBeenCollected()
	{
		m_BeenCollectedByShape = false;
	}

	Vector3 GetDirFromP0ToP1() const
	{
		return (m_P1->GetPoint() - m_P0->GetPoint()).Normalized();
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
		{
			m_P0 = &newPoint;
			return;
		}

		if (&existing == m_P1)
		{
			m_P1 = &newPoint;
			return;
		}

		assert(false);
	}

	bool IsAttachedTo(const Face& f) const
	{
		return (&f == m_Face1) || (&f == m_Face2);
	}

	bool IsAttachedTo(const ShapePoint& p) const
	{
		return (&p == m_P0) || (&p == m_P1);
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
	const Shape* m_OwnerShape;
};
