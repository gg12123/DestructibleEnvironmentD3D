#pragma once
#include "Vector3.h"

struct ContactPointPIP
{
	Vector3 Point;
	int PointIndex;
	int ShapeId; // The id of the shape that contains the point

	ContactPointPIP(const Vector3& p, int pi, int sid)
	{
		Point = p;
		PointIndex = pi;
		ShapeId = sid;
	}
};

struct ContactPointEC
{
	Vector3 Point;
	int EdgeIndexShape1;
	int EdgeIndexShape2;

	ContactPointEC(const Vector3& p, int i1, int i2)
	{
		Point = p;
		EdgeIndexShape1 = i1;
		EdgeIndexShape2 = i2;
	}
};