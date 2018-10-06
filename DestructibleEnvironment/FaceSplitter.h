#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "PoolOfRecyclables.h"
#include "NewEdgePointsCotainer.h"
#include "ToBeNewPoint.h"
#include "FaceFaceIntersection.h"
#include "Polygon2.h"
#include "Polygon2IntersectionFinder.h"
#include "Polygon2Splitter.h"
#include "SplitFaceRegion.h"
#include "FaceRelationshipWithOtherShape.h"
#include "ConvexPolyCreator.h"

class Face;

class FaceSplitter
{
public:
	void SplitOriginalShapesFace(Face& toSplit);
	void SplitCutShapesFace(Face& toSplit);

	void Clear()
	{
		m_NewOutsideFaces.clear();
		m_NewInIntersectionFaces.clear();
	}

	auto& GetNewOutsideFaces()
	{
		return m_NewOutsideFaces;
	}

	auto& GetNewInIntersectionFaces()
	{
		return m_NewInIntersectionFaces;
	}

private:
	void SplitCommon(Face& toSplit);

	void CreateNewPointObjects();
	void ProcessIntersection(const FaceFaceIntersection<Vector2>& inter);
	void ProcessIntersection(const FaceEdgeIntersection<Vector2>& inter, const Vector2& otherFaceNormal, ToBeNewPoint& newPoint, ToBeNewPoint& other);
	void AddPoint(ToBeNewPoint& p);

	ToBeNewPoint* GetNextStartEdgePoint();
	ToBeNewPoint* GetNextStartAcrossPoint();

	ToBeNewPoint& ProcessToNextEdgePointAlongPerimeter(ToBeNewPoint& ep);
	ToBeNewPoint& ProcessToNextEdgePointArossFace(ToBeNewPoint& ep);

	ToBeNewPoint& GetNextPointAcrossFace(ToBeNewPoint& ap);

	void ProcessPointsStartingOnEdge(ToBeNewPoint& startEdgePoint);
	void ProcessPointsAcrossFaceOnly(ToBeNewPoint& startAcrossFacePoint);

	FaceRelationshipWithOtherShape InOrOutForPerminRegion(const ToBeNewPoint& sep); // start edge point
	FaceRelationshipWithOtherShape InOrOutForContainedRegion(const ToBeNewPoint& sap); // start across point

	void CreateSplitFaceRegions();
	SplitFaceRegion& CreateNextReion(FaceRelationshipWithOtherShape inOrOut);

	int NextPerimeterIndex(int index);

	void AssignContainedChildren();
	void FindParent(SplitFaceRegion& child);
	bool RegionIsInsideOther(const SplitFaceRegion& maybeInside, const SplitFaceRegion& other, float& dist);

	void CreateFaces(const SplitFaceRegion& region);
	void CreateInIntersectionOnlyFaces(const SplitFaceRegion& region);

	Face * m_FaceBeingSplit;
	Polygon2* m_CurrentPerimeterPoly;

	std::vector<Face*> m_NewOutsideFaces;
	std::vector<Face*> m_NewInIntersectionFaces;

	std::vector<ToBeNewPoint*> m_PerimeterPoints;
	std::vector<ToBeNewPoint*> m_EdgePoints;
	std::vector<ToBeNewPoint*> m_AcrossPoints;

	std::unique_ptr<PoolOfRecyclables<ToBeNewPoint>> m_NewPointsPool;
	std::unique_ptr<PoolOfRecyclables<Polygon2>> m_PolyPool;
	std::unique_ptr<PoolOfRecyclables<SplitFaceRegion>> m_RegionPool;

	NewEdgePointsContainer m_NewEdgePointsContainer;
	Polygon2IntersectionFinder m_InersectionFinder;
	Polygon2Splitter m_PolySplitter;
	ConvexPolyCreator m_ConvexCreator;

	std::vector<SplitFaceRegion*> m_PerimRegions;
	std::vector<SplitFaceRegion*> m_ContainedRegions;

	std::vector<EdgeCastHit> m_EdgeCastHits;

	int m_CurrentVisitId;
};