#pragma once
#include <array>
#include <vector>
#include <stack>
#include "Shape.h"
#include "Face.h"
#include "PerFaceSplitData.h"

class FaceIterator
{
public:

	void SetShapeToUseNext(Shape& s)
	{
		m_ShapeToUseNext = &s;
	}

	void SetPerFaceData(std::vector<PerFaceSplitData>& perFaceData)
	{
		m_PerFaceData = &perFaceData;
	}

	void CreateShapes(const std::vector<Face*>& faces, std::vector<Shape*>& newShapes)
	{
		auto root = FindNextRootFace(faces);

		while (root)
		{
			newShapes.emplace_back(&CreateShape(*root));
			root = FindNextRootFace(faces);
		}
	}

private:
	Shape & GetNextShapeToUse()
	{
		auto s = m_ShapeToUseNext;
		if (s)
		{
			m_ShapeToUseNext = nullptr;
			return *s;
		}
		// get one from pool
		return *(new Shape());
	}

	Shape& CreateShape(Face& rootFace)
	{
		auto& newShape = GetNextShapeToUse();
		auto& perFaceData = *m_PerFaceData;

		auto rootsRelationship = perFaceData[rootFace.GetIdForSplitter()].RelationshipWithOtherShape;

		newShape.Clear();
		m_FaceStack.push(&rootFace);

		while (!m_FaceStack.empty())
		{
			auto& next = *m_FaceStack.top();
			m_FaceStack.pop();

			auto id = next.GetIdForSplitter();
			auto& nextsData = perFaceData[id];

			nextsData.Visited = true;
			newShape.AddFace(next);

			if (nextsData.RelationshipWithOtherShape != FaceRelationshipWithOtherShape::Unkown)
				assert(nextsData.RelationshipWithOtherShape == rootsRelationship);

			nextsData.RelationshipWithOtherShape = rootsRelationship;

			auto& links = next.GetLinkedFaces();
			for (auto it1 = links.begin(); it1 != links.end(); it1++)
			{
				auto& edgesLinks = *it1;
				for (auto it2 = edgesLinks.begin(); it2 != edgesLinks.end(); it2++)
				{
					auto linkedFace = *it2;
					if (!perFaceData[linkedFace->GetIdForSplitter()].Visited)
						m_FaceStack.push(linkedFace);
				}
			}
		}
		return newShape;
	}

	Face* FindNextRootFace(const std::vector<Face*>& faces)
	{
		auto& perFaceData = *m_PerFaceData;

		for (auto it = faces.begin(); it != faces.end(); it++)
		{
			auto face = *it;
			if (!perFaceData[face->GetIdForSplitter()].Visited)
				return face;
		}
		return nullptr;
	}

	Shape * m_ShapeToUseNext = nullptr;
	std::vector<PerFaceSplitData>* m_PerFaceData = nullptr;
	std::stack<Face*> m_FaceStack;
};
