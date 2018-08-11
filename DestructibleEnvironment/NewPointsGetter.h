// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

class Point;

/**
 * 
 */
class NewPointsGetter
{
public:
	NewPointsGetter();
	~NewPointsGetter();

	void AddPoints(const Point& existing1, const Point& existing2, Point& newAbove, Point& newBelow);
	void AddPoints(const Point& inside, Point& newAbove, Point& newBelow);

	Point& GetPointAbove(const Point& existing1, const Point& existing2) const;
	Point& GetPointBelow(const Point& existing1, const Point& existing2) const;
	Point& GetPointAbove(const Point& inside) const;
	Point& GetPointBelow(const Point& inside) const;
};
