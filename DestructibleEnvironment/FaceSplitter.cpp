#include <assert.h>
#include "FaceSplitter.h"
#include "Face.h"
#include "Shape.h"

void FaceSplitter::AddPoint(ToBeNewPoint& p)
{
	p.TimesAdded++;
	p.LastVisitedId = m_CurrentVisitId;
	
	m_CurrentPerimeterPoly->Add(p.Position);
}

void FaceSplitter::ProcessIntersection(const FaceEdgeIntersection<Vector2>& inter, const Vector2& otherFaceNormal, ToBeNewPoint& newPoint, ToBeNewPoint& other)
{
	if (inter.PiercedFace == m_FaceBeingSplit)
	{
		newPoint.SetupAcrossPoint(inter.Position, otherFaceNormal, other);
		m_AcrossPoints.emplace_back(newPoint);
	}
	else
	{
		newPoint.SetupEdgePoint(inter.Position, otherFaceNormal, other);
		m_EdgePoints.emplace_back(newPoint);
		m_NewEdgePointsContainer.AddEdgePoint(inter.PiercingEdge, newPoint);
	}
}

void FaceSplitter::ProcessIntersection(const FaceFaceIntersection<Vector2>& inter)
{
	auto& p1 = m_NewPointsPool->Recycle();
	auto& p2 = m_NewPointsPool->Recycle();

	auto otherFace = (inter.Face1 == m_FaceBeingSplit) ? inter.Face2 : inter.Face1;

	auto n = m_FaceBeingSplit->GetShape().GetTransform().ToLocalDirection(otherFace->GetNormalWorld());
	auto nFaceSpace = m_FaceBeingSplit->ToFaceSpaceDirection(n);

	ProcessIntersection(inter.Intersection1, nFaceSpace, p1, p2);
	ProcessIntersection(inter.Intersection2, nFaceSpace, p2, p1);
}

void FaceSplitter::CreateNewPointObjects()
{
	m_EdgePoints.clear();
	m_AcrossPoints.clear();
	m_PerimeterPoints.clear();

	m_NewPointsPool->Reset();

	auto& originalPoints = m_FaceBeingSplit->GetFacePoly().GetPoints();
	auto& inters = m_FaceBeingSplit->GetIntersections();

	for (auto it = inters.begin(); it != inters.end(); it++)
		ProcessIntersection(*it);

	auto index = 0;
	auto originalIndex = 0;
	for (auto it = originalPoints.begin(); it != originalPoints.end(); it++)
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

void FaceSplitter::CreateFacesFromSplit(const SplitFaceRegion& region)
{
	auto vec = (region.GetInOrOut() == FaceRelationshipWithOtherShape::InIntersection) ?
		m_NewInIntersectionFaces :
		m_NewOutsideFaces;

	region.CreateFaces(*vec, *m_FaceBeingSplit);
}

void FaceSplitter::SplitFace(Face& toSplit)
{
	SplitCommon(toSplit);

	for (auto it = m_PerimRegions.begin(); it != m_PerimRegions.end(); it++)
		CreateFacesFromSplit(**it);

	for (auto it = m_ContainedRegions.begin(); it != m_ContainedRegions.end(); it++)
		CreateFacesFromSplit(**it);
}

void FaceSplitter::SplitCommon(Face& toSplit)
{
	m_FaceBeingSplit = &toSplit;
	CreateNewPointObjects();
	CreateSplitFaceRegions();
	AssignContainedChildren();

	for (auto it = m_PerimRegions.begin(); it != m_PerimRegions.end(); it++)
		(*it)->GenerateInitialConvexPieces(*m_PolyPool, m_ConvexCreator);

	for (auto it = m_ContainedRegions.begin(); it != m_ContainedRegions.end(); it++)
		(*it)->GenerateInitialConvexPieces(*m_PolyPool, m_ConvexCreator);

	for (auto it = m_PerimRegions.begin(); it != m_PerimRegions.end(); it++)
		(*it)->ClipToContainedChild(m_InersectionFinder, m_PolySplitter, *m_PolyPool);
}

SplitFaceRegion& FaceSplitter::CreateNextReion(FaceRelationshipWithOtherShape inOrOut) // pass in weather the region is in or outside
{
	auto& region = m_RegionPool->Recycle();

	m_CurrentPerimeterPoly->EnsureCorrectWindingDirection();
	region.Init(*m_CurrentPerimeterPoly, inOrOut);

	m_CurrentPerimeterPoly = &m_PolyPool->Recycle();
	return region;
}

void FaceSplitter::AssignContainedChildren()
{
	for (auto it = m_ContainedRegions.begin(); it != m_ContainedRegions.end(); it++)
		FindParent(**it);
}

static bool RegionIsInsideOther1(const SplitFaceRegion& maybeInside, const SplitFaceRegion& other)
{
	auto& testPoints = maybeInside.GetPerimeterPoly().GetPoints();
	auto& otherPoly = other.GetPerimeterPoly();

	auto c = PointInPolyCase::OnBoundary;
	auto it = testPoints.begin();

	while ((c == PointInPolyCase::OnBoundary) && (it != testPoints.end()))
	{
		c = otherPoly.PointIsInside(*it);
		it++;
	}
	assert(c != PointInPolyCase::OnBoundary);
	return (c == PointInPolyCase::Inside);
}

bool FaceSplitter::RegionIsInsideOther(const SplitFaceRegion& maybeInside, const SplitFaceRegion& other, float& dist)
{
	if (RegionIsInsideOther1(maybeInside, other))
	{
		auto origin = maybeInside.GetPerimeterPoly().GetPointAt(0);
		auto dir = Vector2::Up();

		m_EdgeCastHits.clear();
		other.GetPerimeterPoly().RayCastAllEdges(origin, dir, m_EdgeCastHits);

		dist = MathU::Infinity;

		for (auto it = m_EdgeCastHits.begin(); it != m_EdgeCastHits.end(); it++)
		{
			auto d = ((*it).HitPoint - origin).Magnitude();

			if (d < dist)
				dist = d;
		}
		return true;
	}
	return false;
}

void FaceSplitter::UpdateInFindParent(const std::vector<SplitFaceRegion*>& regions, const SplitFaceRegion& child, SplitFaceRegion*& parent, float& closest)
{
	float dist;
	for (auto it = regions.begin(); it != regions.end(); it++)
	{
		auto other = *it;
		if (other == &child)
			continue;

		if (RegionIsInsideOther(child, *other, dist))
		{
			if (dist < closest)
			{
				closest = dist;
				parent = other;
			}
		}
	}
}

void FaceSplitter::FindParent(SplitFaceRegion& child)
{
	auto closest = MathU::Infinity;
	SplitFaceRegion* parent = nullptr;

	UpdateInFindParent(m_ContainedRegions, child, parent, closest);
	UpdateInFindParent(m_PerimRegions, child, parent, closest);

	assert(parent);
	parent->SetContainedChild(child);
}

void FaceSplitter::CreateSplitFaceRegions()
{
	m_ContainedRegions.clear();
	m_PerimRegions.clear();

	m_RegionPool->Reset();
	m_PolyPool->Reset();

	m_CurrentPerimeterPoly = &m_PolyPool->Recycle();
	m_CurrentVisitId = 0;

	auto sep = GetNextStartEdgePoint();
	while (sep)
	{
		ProcessPointsStartingOnEdge(*sep);
		m_PerimRegions.emplace_back(&CreateNextReion(InOrOutForPerminRegion(*sep)));
		m_CurrentVisitId++;
		sep = GetNextStartEdgePoint();
	}

	auto sap = GetNextStartAcrossPoint();
	while (sap)
	{
		ProcessPointsAcrossFaceOnly(*sap);
		m_ContainedRegions.emplace_back(&CreateNextReion(InOrOutForContainedRegion(*sap)));
		m_CurrentVisitId++;
		sap = GetNextStartAcrossPoint();
	}

	if (m_EdgePoints.size() == 0)
	{
		assert(m_ContainedRegions.size() > 0);

		// assign original points to curr poly
		m_PerimRegions.emplace_back(&CreateNextReion(FaceRelationshipWithOtherShape::Unkown)); // the relationship will be given by the contained child
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

FaceRelationshipWithOtherShape FaceSplitter::InOrOutForPerminRegion(const ToBeNewPoint& sep)
{
	auto nextAlongEdge = m_PerimeterPoints[NextPerimeterIndex(sep.Index)];
	return (Vector2::Dot(nextAlongEdge->Position - sep.Position, sep.FaceNormal) < 0.0f) ?
		FaceRelationshipWithOtherShape::InIntersection :
		FaceRelationshipWithOtherShape::NotInIntersection;
}

// the sap was inserted into the current perim poly at index 0
FaceRelationshipWithOtherShape FaceSplitter::InOrOutForContainedRegion(const ToBeNewPoint& sap)
{
	auto a = m_CurrentPerimeterPoly->CalculateSignedArea();

	// TODO - not sure if this is the right way around.
	auto polyEdgeNormal = Polygon2::AreaGivesCorrectWinding(a) ?
		m_CurrentPerimeterPoly->GetNormalAt(0) : 
		-m_CurrentPerimeterPoly->GetNormalAt(0);

	return Vector2::Dot(sap.FaceNormal, polyEdgeNormal) > 0.0f ?
		FaceRelationshipWithOtherShape::InIntersection :
		FaceRelationshipWithOtherShape::NotInIntersection;
}