#include <assert.h>
#include "FaceSplitter.h"
#include "Face.h"

void FaceSplitter::AddPoint(ToBeNewPoint& p)
{
	p.TimesAdded++;
	p.LastVisitedId = m_CurrentVisitId;
	m_FaceCreator.AddPoint(p.Position);
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

	m_CurrentVisitId = 0;
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
	m_FaceBeingSplit = &toSplit;
	CreateNewPointObjects();
	CreateFacesAcrossEntireFaceToSplit();
}

void FaceSplitter::SplitCutShapesFace(Face& toSplit)
{
	m_FaceBeingSplit = &toSplit;
	CreateNewPointObjects();
	CreateFacesInIntersectionOnly();
}

void FaceSplitter::CreateFacesAcrossEntireFaceToSplit()
{
	assert(m_EdgePoints.size() > 0);

	auto sep = GetNextStartEdgePointEntireFace();

	m_FaceCreator.SetNormal(m_FaceBeingSplit->GetNormal());

	while (sep)
	{
		m_FaceCreator.Restart();
		ProcessPointsStartingOnEdge(*sep);

		auto& faces = DefinesStartOfFaceInIntersection(*sep) ? m_NewInsideFaces : m_NewOutsideFaces;
		m_FaceCreator.CreateFaces(faces);
		m_CurrentVisitId++;

		sep = GetNextStartEdgePointEntireFace();
	}
}

ToBeNewPoint* FaceSplitter::GetNextStartAcrossPointIntersectionOnly()
{
	for (auto it = m_AcrossPoints.begin(); it != m_AcrossPoints.end(); it++)
	{
		if ((*it)->TimesAdded == 0)
			return *it;
	}
	return nullptr;
}

ToBeNewPoint* FaceSplitter::GetNextStartEdgePointIntersectionOnly()
{
	for (auto it = m_EdgePoints.begin(); it != m_EdgePoints.end(); it++)
	{
		auto ep = *it;

		if ((ep->TimesAdded == 0) && DefinesStartOfFaceInIntersection(*ep))
			return ep;
	}
	return nullptr;
}

ToBeNewPoint* FaceSplitter::GetNextStartEdgePointEntireFace()
{
	for (auto it = m_EdgePoints.begin(); it != m_EdgePoints.end(); it++)
	{
		auto ep = *it;

		if ((ep->TimesAdded < 2) && !ep->BeenStartedFrom)
			return ep;
	}
	return nullptr;
}

void FaceSplitter::CreateFacesInIntersectionOnly()
{
	m_FaceCreator.SetNormal(m_FaceBeingSplit->GetNormal());

	auto sep = GetNextStartEdgePointIntersectionOnly();
	while (sep)
	{
		m_FaceCreator.Restart();
		ProcessPointsStartingOnEdge(*sep);

		m_FaceCreator.CreateFaces(m_NewInsideFaces);
		m_FaceCreator.CreateUpsideDownFaces(m_NewOutsideFaces);
		m_CurrentVisitId++;

		sep = GetNextStartEdgePointIntersectionOnly();
	}

	auto sap = GetNextStartAcrossPointIntersectionOnly();
	while (sap)
	{
		m_FaceCreator.Restart();
		ProcessPointsAcrossFaceOnly(*sap);

		m_FaceCreator.CreateFaces(m_NewInsideFaces);
		m_FaceCreator.CreateUpsideDownFaces(m_NewOutsideFaces);
		m_CurrentVisitId++;

		sap = GetNextStartAcrossPointIntersectionOnly();
	}
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