#include <assert.h>
#include "FaceSplitter.h"
#include "Face.h"

void FaceSplitter::AddPoint(ToBeNewPoint& p)
{
	p.TimesAdded++;
	p.LastVisitedId = m_CurrentVisitId;
	
	// add to current perim poly
}

void FaceSplitter::ProcessIntersection(const FaceFaceIntersection& inter)
{
	auto& p1 = m_NewPointsPool->Recycle();
	auto& p2 = m_NewPointsPool->Recycle();

	if (inter.PiercedFace1 == m_FaceBeingSplit)
	{
		p1.SetupAcrossPoint(inter.Position1, p2);
		m_AcrossPoints.emplace_back(p1);
	}
	else
	{
		p1.SetupEdgePoint(inter.Position1, inter.PiercedFace1->GetNormal(), p2);
		m_EdgePoints.emplace_back(p1);
		m_NewEdgePointsContainer.AddEdgePoint(inter.PiercingEdge1, p1);
	}

	if (inter.PiercedFace2 == m_FaceBeingSplit)
	{
		p2.SetupAcrossPoint(inter.Position2, p1);
		m_AcrossPoints.emplace_back(p2);
	}
	else
	{
		p2.SetupEdgePoint(inter.Position2, inter.PiercedFace2->GetNormal(), p1);
		m_EdgePoints.emplace_back(p2);
		m_NewEdgePointsContainer.AddEdgePoint(inter.PiercingEdge2, p2);
	}
}

void FaceSplitter::CreateNewPointObjects()
{
	m_EdgePoints.clear();
	m_AcrossPoints.clear();
	m_PerimeterPoints.clear();

	m_NewPointsPool->Reset();

	auto originalPoints = m_FaceBeingSplit->GetCachedPoints();
	auto inters = m_FaceBeingSplit->GetIntersections();

	for (auto it = inters.Begin(); it != inters.End(); it++)
		ProcessIntersection(*it);

	auto index = 0;
	auto originalIndex = 0;
	for (auto it = originalPoints.Begin(); it != originalPoints.End(); it++)
	{
		auto& pOrig = m_NewPointsPool->Recycle();
		pOrig.SetupOriginal(*it);
		pOrig.Index = index;
		index++;

		m_PerimeterPoints.emplace_back(&pOrig);

		if (m_NewEdgePointsContainer.HasNewEdgePointsAt(originalIndex))
		{
			m_NewEdgePointsContainer.StartReturningEdgesFrom(originalIndex, *it);
			auto ep = m_NewEdgePointsContainer.GetNextNewEdgePoint();

			while (ep)
			{
				ep->Index = index;
				index++;
				m_PerimeterPoints.emplace_back(ep);
				ep = m_NewEdgePointsContainer.GetNextNewEdgePoint();
			}
		}

		originalIndex++;
	}
}

void FaceSplitter::ProcessPointsStartingOnEdge(ToBeNewPoint& startEdgePoint)
{
	startEdgePoint.BeenStartedFrom = true;
	auto alongPerim = true;

	auto ep = &startEdgePoint;

	do
	{
		if (alongPerim)
		{
			ep = &ProcessToNextEdgePointAlongPerimeter(*ep);
			alongPerim = false;
		}
		else
		{
			ep = &ProcessToNextEdgePointArossFace(*ep);
			alongPerim = true;
		}
	} while (ep != &startEdgePoint);
}

void FaceSplitter::SplitOriginalShapesFace(Face& toSplit)
{
	SplitCommon(toSplit);
}

void FaceSplitter::SplitCutShapesFace(Face& toSplit)
{
	SplitCommon(toSplit);
}

void FaceSplitter::SplitCommon(Face& toSplit)
{
	m_FaceBeingSplit = &toSplit;
	CreateNewPointObjects();
	CreateSplitFaceRegions();
	AssignContainedChildren();

	for (auto it = m_Regions.begin(); it != m_Regions.end(); it++)
		(*it)->GenerateInitialConvexPieces(*m_PolyPool, m_ConvexCreator);

	for (auto it = m_Regions.begin(); it != m_Regions.end(); it++)
	{
		auto &r = **it;
		if (!r.HasParent())
			r.ClipToContainedChild(m_InersectionFinder, m_PolySplitter, *m_PolyPool);
	}
}

SplitFaceRegion& FaceSplitter::CreateNextReion(FaceRelationshipWithOtherShape inOrOut) // pass in weather the region is in or outside
{
	auto& region = m_RegionPool->Recycle();
	region.Init(*m_CurrentPerimeterPoly);
	m_Regions.emplace_back(&region);
	m_CurrentPerimeterPoly = &m_PolyPool->Recycle();
	return region;
}

void FaceSplitter::AssignContainedChildren()
{
	for (auto it = m_RegionsWithParents.begin(); it != m_RegionsWithParents.end(); it++)
		FindParent(**it);
}

void FaceSplitter::FindParent(SplitFaceRegion& child)
{
	// cast out to other regions and use the first hit
}

void FaceSplitter::CreateSplitFaceRegions()
{
	m_Regions.clear();
	m_RegionsWithParents.clear();

	m_RegionPool->Reset();
	m_PolyPool->Reset();

	m_CurrentPerimeterPoly = &m_PolyPool->Recycle();
	m_CurrentVisitId = 0;

	auto sep = GetNextStartEdgePoint();
	while (sep)
	{
		ProcessPointsStartingOnEdge(*sep);
		CreateNextReion(DefinesStartOfFaceInIntersection(sep*) ? FaceRelationshipWithOtherShape::InIntersection : FaceRelationshipWithOtherShape::NotInIntersection);
		m_CurrentVisitId++;
		sep = GetNextStartEdgePoint();
	}

	auto sap = GetNextStartAcrossPoint();
	while (sap)
	{
		ProcessPointsAcrossFaceOnly(*sap);
		m_RegionsWithParents.emplace_back(CreateNextReion()); // determin relationship by casting from an edge in the normal direction and counting intersections
		m_CurrentVisitId++;
		sap = GetNextStartAcrossPoint();
	}

	if (m_EdgePoints.size() == 0)
	{
		// assign original points to curr poly
		CreateNextReion(FaceRelationshipWithOtherShape::Unkown); // the relationship will be given by the contained child
	}
}

ToBeNewPoint* FaceSplitter::GetNextStartAcrossPoint()
{
	for (auto it = m_AcrossPoints.begin(); it != m_AcrossPoints.end(); it++)
	{
		if ((*it)->TimesAdded == 0)
			return *it;
	}
	return nullptr;
}

ToBeNewPoint* FaceSplitter::GetNextStartEdgePoint()
{
	for (auto it = m_EdgePoints.begin(); it != m_EdgePoints.end(); it++)
	{
		auto ep = *it;

		if ((ep->TimesAdded < 2) && !ep->BeenStartedFrom)
			return ep;
	}
	return nullptr;
}

ToBeNewPoint& FaceSplitter::ProcessToNextEdgePointAlongPerimeter(ToBeNewPoint& ep)
{
	auto i = ep.Index;

	do
	{
		AddPoint(*m_PerimeterPoints[i]);
		i = NextPerimeterIndex(i);
	} while (m_PerimeterPoints[i]->Type != NewPointType::Edge);

	return *m_PerimeterPoints[i];
}

ToBeNewPoint& FaceSplitter::ProcessToNextEdgePointArossFace(ToBeNewPoint& ep)
{
	AddPoint(ep);
	auto p = ep.Other;

	while (p->Type != NewPointType::Edge)
	{
		AddPoint(*p);
		p = &GetNextPointAcrossFace(*p);
	}
}

ToBeNewPoint& FaceSplitter::GetNextPointAcrossFace(ToBeNewPoint& ap)
{
	auto smallestDist = MathU::Infinity;
	ToBeNewPoint* closest = nullptr;

	// TODO - OPTIMISE

	for (auto it = m_AcrossPoints.begin(); it != m_AcrossPoints.end(); it++)
	{
		auto p = *it;

		if (p != &ap && (p->LastVisitedId != m_CurrentVisitId))
		{
			auto dist = (ap.Position - p->Position).Magnitude();

			if (dist < smallestDist)
			{
				smallestDist = dist;
				closest = p;
			}
		}
	}
	return *closest->Other;
}

void FaceSplitter::ProcessPointsAcrossFaceOnly(ToBeNewPoint& startAcrossFacePoint)
{
	auto p = &startAcrossFacePoint;

	do
	{
		AddPoint(*p);
		p = &GetNextPointAcrossFace(*p);
	} while (p != &startAcrossFacePoint);
}

int FaceSplitter::NextPerimeterIndex(int index)
{
	return (index + 1) % m_PerimeterPoints.size();
}

bool FaceSplitter::DefinesStartOfFaceInIntersection(const ToBeNewPoint& sep)
{
	auto nextAlongEdge = m_PerimeterPoints[NextPerimeterIndex(sep.Index)];
	return (Vector3::Dot(nextAlongEdge->Position - sep.Position, sep.FaceNormal) < 0.0f);
}