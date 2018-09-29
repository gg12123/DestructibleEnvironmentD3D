#pragma once
#include "Vector3.h"

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

	Vector3 Position;
	Vector3 FaceNormal;

	int LastVisitedId;

	void SetupEdgePoint(const Vector3& pos, const Vector3& normal, ToBeNewPoint& other)
	{
		Type = NewPointType::Edge;
		Position = pos;
		FaceNormal = normal;
		Other = &other;
		TimesAdded = 0;
		BeenStartedFrom = false;
	}

	void SetupAcrossPoint(const Vector3& pos, ToBeNewPoint& other)
	{
		Type = NewPointType::AcrossFace;
		Position = pos;
		Other = &other;
		TimesAdded = 0;
		LastVisitedId = 0;
	}

	void SetupOriginal(const Vector3& pos)
	{
		Type = NewPointType::Edge;
		Position = pos;
		TimesAdded = 0;
	}
};
