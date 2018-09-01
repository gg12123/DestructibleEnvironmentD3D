// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include <vector>
#include <assert.h>

enum class PointPlaneRelationship
{
	Above,
	Below,
	Inside
};

class Shape;
class NewPointsGetter;

/**
 * 
 */
class Point
{
public:
	static constexpr float PointInPlaneTol = 0.0001f;

	Point(const Vector3& point)
	{
		m_Point = point;
	}

	~Point()
	{
	}

	PointPlaneRelationship GetPlaneRelationship() const
	{
		return m_PlaneRelationship;
	}

	void SetPlaneRelationship(PointPlaneRelationship relationship)
	{
		m_PlaneRelationship = relationship;
	}

	Vector3 GetPoint() const
	{
		return m_Point;
	}

	void SetPoint(const Vector3& p)
	{
		m_Point = p;
	}

	int GetId() const
	{
		return m_Id;
	}

	void SetId(int id)
	{
		m_Id = id;
	}

	void CentreAndCache(const Vector3& centre, std::vector<Vector3>& points)
	{
		m_Point -= centre;
		points.emplace_back(m_Point);
	}

	Point* GetLinkedPoint1() const
	{
		return m_LinkedPoint1;
	}

	Point* GetLinkedPoint2() const
	{
		return m_LinkedPoint2;
	}

	Point* NextLinkedPoint(Point &prev) const
	{
		if (m_LinkedPoint1 == &prev)
		{
			return m_LinkedPoint2;
		}

		else if (m_LinkedPoint2 == &prev)
		{
			return m_LinkedPoint1;
		}

		assert(false);
		return nullptr;
	}

	void AddLink(Point& linkedPoint)
	{
		if (m_LinkedPoint1 == nullptr)
		{
			m_LinkedPoint1 = &linkedPoint;
			return;
		}

		else if (m_LinkedPoint2 == nullptr)
		{
			m_LinkedPoint2 = &linkedPoint;
			return;
		}

		assert(false);
		return;
	}

	void Split(const Vector3& P0, const Vector3& n, NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow, int& numInside);

	static bool PointsBridgePlane(Point &p1, Point& p2)
	{
		if (p1.GetPlaneRelationship() == PointPlaneRelationship::Above && p2.GetPlaneRelationship() == PointPlaneRelationship::Below)
			return true;

		if (p2.GetPlaneRelationship() == PointPlaneRelationship::Above && p1.GetPlaneRelationship() == PointPlaneRelationship::Below)
			return true;

		return false;
	}

	static bool BothAbove(const Point& p1, const Point& p2)
	{
		return p1.GetPlaneRelationship() == PointPlaneRelationship::Above && p2.GetPlaneRelationship() == PointPlaneRelationship::Above;
	}

	static bool BothBelow(const Point& p1, const Point& p2)
	{
		return p1.GetPlaneRelationship() == PointPlaneRelationship::Below && p2.GetPlaneRelationship() == PointPlaneRelationship::Below;
	}

	static bool BothInside(const Point& p1, const Point& p2)
	{
		return p1.GetPlaneRelationship() == PointPlaneRelationship::Inside && p2.GetPlaneRelationship() == PointPlaneRelationship::Inside;
	}

private:
	Point * m_LinkedPoint1 = nullptr;
	Point * m_LinkedPoint2 = nullptr;

	Vector3 m_Point;

	int m_Id;

	PointPlaneRelationship m_PlaneRelationship;
};
