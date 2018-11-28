#pragma once
#include <vector>
#include <array>
#include "Face.h"
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "Vector3.h"

class ConcaveFace
{
private:
	void DetachOriginalFace(const Face& orig)
	{
		auto edges = orig.GetEdgeObjects();
		for (auto it = edges.begin(); it != edges.end(); it++)
			(*it)->DeRegisterFace(orig);
	}

	Face& CreateFace(int i0, int i1, int i2)
	{
		// The indexs key into the vectors.

		// If, for example, i1 is next of i0, then the edge between i0 and i1 can come from the vectors.
		// Otherwise, it may need to be created and put into the map, or just gotten from the map.

		// Two of the edges will be in the vectors, I think, so calculate the third direction from these two directions in a stable way.
	}

public:
	void Clear()
	{
		m_Points.clear();
		m_Edges.clear();
		m_Dirs.clear();
	}

	void AddPoint(ShapePoint& point, const Vector3& dirToNext, ShapeEdge& edgeToNext)
	{
		m_Points.emplace_back(&point);
		m_Edges.emplace_back(&edgeToNext);
		m_Dirs.emplace_back(dirToNext);
	}

	void Triangulate(const Face& original, std::vector<Face*>& triangleFaces)
	{
		DetachOriginalFace(original);
	}

	void Init(ShapeEdgesCreator& edges)
	{
		m_NewEdges = &edges;
	}

private:
	ShapeEdgesCreator * m_NewEdges = nullptr;

	std::vector<ShapePoint*> m_Points;
	std::vector<ShapeEdge*> m_Edges;
	std::vector<Vector3> m_Dirs;
};
