// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include "Vector2.h"
#include "FaceFaceIntersection.h"
#include "Polygon2.h"
#include "PoolOfRecyclables.h"
#include <vector>
#include <memory>

class Shape;

class LinkedNeighbour
{
public:
	LinkedNeighbour(Face& neighbour, int edgeOnNeighbour)
	{
		m_Neighbour = &neighbour;
		m_EdgeOnNeighbour = edgeOnNeighbour;
	}

	Face& GetNeighbour() const
	{
		return *m_Neighbour;
	}

	int GetEdgeOnNeighbour() const
	{
		return m_EdgeOnNeighbour;
	}

private:
	Face * m_Neighbour;
	int m_EdgeOnNeighbour;
};

/**
 * 
 */
class Face
{
public:
	Face()
	{
		// allocate space in vectors
	}

	Vector2 ToFaceSpaceDirection(const Vector3& shapesSpaceDir) const
	{

	}

	Vector2 ToFaceSpacePosition(const Vector3& shapesSpacePos) const
	{

	}

	Vector3 ToShapeSpacePosition(const Vector2& faceSpacePos) const
	{

	}

	void SetNormal(const Vector3& normal)
	{
		m_Normal = normal;
	}

	auto GetNormal() const
	{
		return m_Normal;
	}

	Vector3 GetNormalWorld() const
	{
	}

	auto GetPlaneP0() const
	{
		return m_CachedPoints[0];
	}

	void RegisterIntersection(const FaceFaceIntersection<Vector3>& inter);

	const auto& GetIntersections()
	{
		return m_Intersections;
	}

	bool HasRegisteredIntersections() const
	{
		return m_Intersections.size() > 0U;
	}

	const auto& GetCachedPoints() const
	{
		return m_CachedPoints;
	}

	auto& GetShape()
	{
		return *m_OwnerShape;
	}

	const auto& GetFacePoly()
	{
		return m_FacePoly;
	}

	const auto& GetSharedPoints() const
	{
		return m_ToSharedPoints;
	}

	const auto& GetLinkedFaces() const
	{
		// TODO - this allows calling code to add and remove links so needs changing.
		return *m_LinkedFaces;
	}

	void AddLink(int edgeOnSelf, Face& link, int linksEdge);

	void DetachLinks();

	void AddPoint(const Vector3& p)
	{
		// TODO - calculate weight ????
		m_CachedPoints.emplace_back(p);
		m_LinkedFaces->Recycle().clear();
	}

	void StartAddingCenteredPoints(const Vector3& normal, const Vector3& firstPoint, int firstSharedPointIndex)
	{
		AddPoint(firstPoint);
		m_Normal = normal;
		m_ToSharedPoints.emplace_back(firstSharedPointIndex);
		m_FacePoly.Add(Vector2::Zero());
		InitFaceCoOrdinateSystem(m_CachedPoints[0]);
	}

	void AddCenteredPoint(const Vector3& point, int sharedPointIndex)
	{
		AddPoint(point);
		m_ToSharedPoints.emplace_back(sharedPointIndex);
		m_FacePoly.Add(ToFaceSpacePosition(point));
	}

	void Clear()
	{
		m_ToSharedPoints.clear();
		m_CachedPoints.clear();
		m_Intersections.clear();
		m_LinkedFaces->Reset();
		m_FacePoly.Clear();
	}

	void ClearRegisteredIntersections()
	{
		m_Intersections.clear();
	}

	bool PointIsInsideFace(const Vector3& pointShapesSpace) const;

	void ReCentre(const Vector3& centre, Shape& owner);

	int GetIdForSplitter() const
	{
		return m_IdForSplitter;
	}

	void SetIdForSplitter(int id)
	{
		m_IdForSplitter = id;
	}

private:
	void InitFaceCoOrdinateSystem(const Vector3& origin);

	bool AlreadyLinkedTo(const Face& neighbour, int edgeOnSelf)
	{
		auto& edgesLinks = m_LinkedFaces->At(edgeOnSelf);
		for (auto it = edgesLinks.begin(); it != edgesLinks.end(); it++)
		{
			if (&neighbour == &(*it).GetNeighbour())
				return true;
		}
		return false;
	}

	void RemoveLink(Face& neighbour, int edgeOnSelf)
	{
		auto& links = m_LinkedFaces->At(edgeOnSelf);

		for (auto it = links.begin(); it != links.end(); it++)
		{
			if (&(*it).GetNeighbour() == &neighbour)
			{
				links.erase(it);
				return;
			}
		}
		assert(false);
	}

	// all points related collections must be parralel.

	std::vector<int> m_ToSharedPoints;
	std::vector<Vector3> m_CachedPoints;

	std::unique_ptr<PoolOfRecyclables<std::vector<LinkedNeighbour>>> m_LinkedFaces;

	std::vector<FaceFaceIntersection<Vector2>> m_Intersections;
	Shape* m_OwnerShape;
	Polygon2 m_FacePoly;

	Vector3 m_Normal;

	int m_IdForSplitter;
};
