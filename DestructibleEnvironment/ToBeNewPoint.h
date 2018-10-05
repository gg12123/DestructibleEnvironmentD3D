#pragma once
#include "Vector2.h"

enum class NewPointType
{
	FromOriginal,
	Edge,
	AcrossFace
};

class ToBeNewPoint
{
public:
	NewPointType Type;
	int Index;
	ToBeNewPoint* Other;
	int TimesAdded;
	bool BeenStartedFrom;

	Vector2 Position;
	Vector2 FaceNormal;

	int LastVisitedId;

	void SetupEdgePoint(const Vector2& pos, const Vector2& normal, ToBeNewPoint& other)
	{
		Type = NewPointType::Edge;
		Position = pos;
		FaceNormal = normal;
		Other = &other;
		TimesAdded = 0;
		BeenStartedFrom = false;
	}

	void SetupAcrossPoint(const Vector2& pos, const Vector2& normal, ToBeNewPoint& other)
	{
		Type = NewPointType::AcrossFace;
		Position = pos;
		Other = &other;
		FaceNormal = normal;
		TimesAdded = 0;
		LastVisitedId = 0;
	}

	void SetupOriginal(const Vector2& pos)
	{
		Type = NewPointType::Edge;
		Position = pos;
		TimesAdded = 0;
	}
};
