#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "PoolOfRecyclables.h"
#include "NewEdgePointsCotainer.h"
#include "ToBeNewPoint.h"
#include "ConvexFaceCreator.h"
#include "FaceFaceIntersection.h"

class Face;

class FaceSplitter
{
public:
	void SplitOriginalShapesFace(Face& toSplit);
	void SplitCutShapesFace(Face& toSplit);

private:
	void CreateNewPointObjects();
	void ProcessIntersection(const FaceFaceIntersection& inter);
	void AddPoint(ToBeNewPoint& p);

	ToBeNewPoint* GetNextStartEdgePointEntireFace();
	ToBeNewPoint* GetNextStartEdgePointIntersectionOnly();
	ToBeNewPoint* GetNextStartAcrossPointIntersectionOnly();

	ToBeNewPoint& ProcessToNextEdgePointAlongPerimeter(ToBeNewPoint& ep);
	ToBeNewPoint& ProcessToNextEdgePointArossFace(ToBeNewPoint& ep);

	ToBeNewPoint& GetNextPointAcrossFace(ToBeNewPoint& ap);

	void ProcessPointsStartingOnEdge(ToBeNewPoint& startEdgePoint);
	void ProcessPointsAcrossFaceOnly(ToBeNewPoint& startAcrossFacePoint);

	bool DefinesStartOfFaceInIntersection(const ToBeNewPoint& sep);

	void CreateFacesAcrossEntireFaceToSplit();
	void CreateFacesInIntersectionOnly();

	int NextPerimeterIndex(int index);

	Face * m_FaceBeingSplit;

	std::vector<Face*> m_NewOutsideFaces;
	std::vector<Face*> m_NewInsideFaces;

	std::vector<ToBeNewPoint*> m_PerimeterPoints;
	std::vector<ToBeNewPoint*> m_EdgePoints;
	std::vector<ToBeNewPoint*> m_AcrossPoints;

	std::unique_ptr<PoolOfRecyclables<ToBeNewPoint>> m_NewPointsPool;

	NewEdgePointsContainer m_NewEdgePointsContainer;
	ConvexFaceCreator m_FaceCreator;

	int m_CurrentVisitId;
};