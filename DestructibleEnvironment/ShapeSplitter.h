#pragma once
#include <vector>
#include "Shape.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "ShapePoint.h"
#include "FaceSplitter.h"
#include "CleanIntersectionFinder.h"
#include "FaceIterator.h"
#include "ShapeElementPool.h"
#include "RedundantPointRemover.h"
#include "FaceTriangulator.h"
#include "NewShapeGeometryCreator.h"
#include "InPlaneFacesCreator.h"
#include "Random.h"

template<class Tshape>
class ShapeSplitter
{
private:
	void ResetHashCounters()
	{
		ObjectWithHash<Face>::ResetNextHashCounter();
		ObjectWithHash<ShapePoint>::ResetNextHashCounter();
		ObjectWithHash<ShapeEdge>::ResetNextHashCounter();
	}

	void CreateNewShapes(Tshape& original, std::vector<Tshape*>& newShapes)
	{
		m_FaceIterator.SetShapeToUseNext(original);
		m_FaceIterator.CreateShapes(m_NewAboveFaces, newShapes);
		m_FaceIterator.CreateShapes(m_NewBelowFaces, newShapes);
	}

	void CleanGeometry(const std::vector<Tshape*>& newShapes) const
	{
		// TODO
	}

	void InitNewShapes(Shape& original, const std::vector<Tshape*>& newShapes)
	{
		// Must copy the transform.
		// Copy is needed becasue the original shape is re-used in the
		// new shapes.
		auto t = original.GetTransform();

		for (auto it = newShapes.begin(); it != newShapes.end(); it++)
			(*it)->OnAllFacesAdded(t);
	}

	bool FindIntersections(const Shape& originalShape, const Plane& sp)
	{
		m_Intersections.clear();
		return m_IntersectionFinder.FindCleanIntersections(originalShape, sp, m_Intersections);
	}

	void CreateNewGeometry()
	{
		m_NewGeometryCreator.Init(m_IntersectionFinder.GetPointPlaneMap());

		for (auto l : m_Intersections)
			m_NewGeometryCreator.CreateGeometry(*l);
	}

	void SplitFaces()
	{
		m_FaceSplitter.Init(m_NewGeometryCreator.GetNewPointMap(), m_NewGeometryCreator.GetNewEdgeMap(), m_IntersectionFinder.GetPointPlaneMap());

		m_NewAboveFaces.clear();
		m_NewBelowFaces.clear();

		for (auto l : m_Intersections)
			m_FaceSplitter.Split(*l, m_NewAboveFaces, m_NewBelowFaces);
	}

	void CreateInPlaneFaces(const Plane& sp)
	{
		m_InPlaneFaceCreator.Init(m_NewGeometryCreator.GetNewPointMap(), m_NewGeometryCreator.GetNewEdgeMap());

		for (auto l : m_Intersections)
			m_InPlaneFaceCreator.Create(*l, sp, m_NewAboveFaces, m_NewBelowFaces);
	}

	Plane CalculateSplitPlane(const Vector3& splitPointLocal) const
	{
		auto a = Random::Range(0.01f, 1.0f);
		auto b = Random::Range(0.01f, 1.0f);
		auto c = Random::Range(0.01f, 1.0f);

		a *= Random::Range(0.0f, 1.0f) > 0.5f ? -1.0f : 1.0f;
		b *= Random::Range(0.0f, 1.0f) > 0.5f ? -1.0f : 1.0f;
		c *= Random::Range(0.0f, 1.0f) > 0.5f ? -1.0f : 1.0f;

		return Plane(Vector3(a, b, c).Normalized(), splitPointLocal);
	}

	void ReturnStuffToPool()
	{
		for (auto l : m_Intersections)
		{
			for (auto i = 0; i < l->GetCount(); i++)
			{
				FacePool::Return(l->GetFaceEntered(i));
				EdgePool::Return(l->GetPiercingEdge(i));
			}
		}
	}

public:
	bool Split(const Vector3& splitPointWorld, Tshape& originalShape, std::vector<Tshape*>& newShapes)
	{
		ResetHashCounters();
		// Assume hashes on all shape elements are reset.

		auto sp = CalculateSplitPlane(originalShape.GetTransform().ToLocalPosition(splitPointWorld));

		if (!FindIntersections(originalShape, sp))
		{
			// log error
			return false;
		}

		CreateNewGeometry();
		SplitFaces();
		CreateInPlaneFaces(sp);
		CreateNewShapes(originalShape, newShapes);
		CleanGeometry(newShapes);

		// This will reset hashes on all shape elements
		InitNewShapes(originalShape, newShapes);

		ReturnStuffToPool();
		return true;
	}

private:
	FaceSplitter m_FaceSplitter;
	FaceIterator<Tshape> m_FaceIterator;
	CleanIntersectionFinder m_IntersectionFinder;
	InPlaneFaceCreator m_InPlaneFaceCreator;
	NewShapeGeometryCreator m_NewGeometryCreator;

	std::vector<IntersectionLoop*> m_Intersections;

	std::vector<Face*> m_NewAboveFaces;
	std::vector<Face*> m_NewBelowFaces;
};
