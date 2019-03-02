// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include "ObjectWithHash.h"
#include <vector>
#include <assert.h>

class Shape;

/**
 * 
 */
class ShapePoint : public ObjectWithHash<ShapePoint>
{
public:
	ShapePoint()
	{
	}

	void OnTakenFromPool(const Vector3& p)
	{
		m_Point = p;
		m_OwnerShape = nullptr;
		m_BeenCollectedByShape = false;
		ResetHash();
	}

	void OnReturnedToPool() {}

	void SetBeenCollected()
	{
		m_BeenCollectedByShape = true;
	}

	void ClearBeenCollected()
	{
		m_BeenCollectedByShape = false;
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

	void SetIndexInShape(int val)
	{
		m_IndexInShape = val;
	}

	int GetIndexInShape() const
	{
		return m_IndexInShape;
	}

	void ReCentre(const Vector3& centre)
	{
		m_Point -= centre;
	}

	const auto& GetPoint() const
	{
		return m_Point;
	}

private:
	Vector3 m_Point;
	bool m_BeenCollectedByShape = false;
	int m_IndexInShape;
	const Shape* m_OwnerShape;
};
