#pragma once
#include <stack>
#include <assert.h>
#include "Shape.h"
#include "Face.h"

template<class Tshape>
class FaceIterator
{
private:
	bool FaceIsVisited(Face& f)
	{
		f.TryAssignHash();
		return m_Visited[f.GetHash()];
	}

	void Visit(Face& f)
	{
		assert(!FaceIsVisited(f));
		m_Visited[f.GetHash()] = true;
	}

	Tshape & GetNextShapeToUse()
	{
		auto s = m_ShapeToUseNext;
		if (s)
		{
			m_ShapeToUseNext = nullptr;
			return *s;
		}
		return *(new Tshape());
	}

	Tshape& CreateShape(Face& rootFace)
	{
		auto& newShape = GetNextShapeToUse();
		newShape.Clear();

		m_FaceStack.push(&rootFace);

		while (!m_FaceStack.empty())
		{
			auto& next = *m_FaceStack.top();
			m_FaceStack.pop();

			if (FaceIsVisited(next))
				continue;

			Visit(next);
			newShape.AddFace(next);

			auto& edges = next.GetEdgeObjects();

			for (auto it = edges.begin(); it != edges.end(); it++)
			{
				auto& other = (*it)->GetOther(next);

				if (!FaceIsVisited(other))
					m_FaceStack.push(&other);
			}
		}
		return newShape;
	}

	Face* FindNextRootFace(const std::vector<Face*>& faces)
	{
		for (auto it = faces.begin(); it != faces.end(); it++)
		{
			auto face = *it;
			if (!FaceIsVisited(*face))
				return face;

		}
		return nullptr;
	}

public:
	void SetShapeToUseNext(Tshape& s)
	{
		m_ShapeToUseNext = &s;
	}

	void CreateShapes(const std::vector<Face*>& faces, std::vector<Tshape*>& newShapes)
	{
		m_Visited.Zero();

		auto root = FindNextRootFace(faces);

		while (root)
		{
			newShapes.emplace_back(&CreateShape(*root));
			root = FindNextRootFace(faces);
		}
	}

private:
	Tshape * m_ShapeToUseNext = nullptr;
	std::stack<Face*> m_FaceStack;
	DynamicArray<int> m_Visited;
};
