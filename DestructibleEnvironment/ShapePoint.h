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

	void SetBeenCollected(bool val)
	{
		m_BeenCollectedByShape = val;
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
};
