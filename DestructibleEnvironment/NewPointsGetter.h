// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "TwoDArray.h"

class Point;

struct PointPair
{
	Point* Above = nullptr;
	Point* Below = nullptr;

	void Set(Point* above, Point* below)
	{
		Above = above;
		Below = below;
	}
};

/**
 * 
 */
class NewPointsGetter
{
public:
	static int constexpr MaxNumPoints = 50;

	NewPointsGetter()
	{
	}

	~NewPointsGetter()
	{
	}

	void AddPoints(const Point& existing1, const Point& existing2, Point& newAbove, Point& newBelow);
	void AddPoints(const Point& inside, Point& newAbove, Point& newBelow);

	Point& GetPointAbove(const Point& existing1, const Point& existing2);
	Point& GetPointBelow(const Point& existing1, const Point& existing2);
	Point& GetPointAbove(const Point& inside);
	Point& GetPointBelow(const Point& inside);

private:
	PointPair & Get(const Point& existing1, const Point& existing2);

	TwoDArray<MaxNumPoints, MaxNumPoints, PointPair> m_PointsAlongEdges;
	std::array<PointPair, MaxNumPoints> m_PointsOnPoints;
};
