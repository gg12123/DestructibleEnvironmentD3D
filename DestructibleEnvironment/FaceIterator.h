#pragma once
#include <array>
#include <vector>
#include <stack>
#include <assert.h>
#include "Shape.h"
#include "Face.h"
#include "MapToFaceRelationship.h"

template<class Tshape>
class FaceIterator
{
private:
	bool FaceIsVisited(const Face& f) const
	{
		// TODO - using the hash like this is not very safe.
		return f.HashIsAssigned();
	}

	void Visit(Face& f)
	{
		assert(!FaceIsVisited(f));
		f.AssignHash();
		m_Map.SetRelationship(f, m_CurrRelationship);
	}

	Tshape & GetNextShapeToUse()
	{
		auto s = m_ShapeToUseNext;
		if (s)
		{
			m_ShapeToUseNext = nullptr;
			return *s;
		}
		// get one from pool
		return *(new Tshape());
	}

	Tshape& CreateShape(Face& rootFace)
	{
		auto& newShape = GetNextShapeToUse();

		auto rootsRelationship = m_Map.GetRelationship(rootFace);

		newShape.Clear();
		m_FaceStack.push(&rootFace);

		while (!m_FaceStack.empty())
		{
			auto& next = *m_FaceStack.top();
			m_FaceStack.pop();

			Visit(next);
			newShape.AddFace(next);

			auto& edges = next.GetEdgeObjects();

			for (auto it = edges.begin(); it != edges.end(); it++)
			{
				auto& other = (*it)->GetOther(next);

				if (!FaceIsVisited(other))
				{
					m_FaceStack.push(&other);
				}
				else
				{
					assert(m_Map.GetRelationship(other) == m_CurrRelationship);
				}
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
			{
				Visit(*face);
				return face;
			}
		}
		return nullptr;
	}

public:
	void SetShapeToUseNext(Tshape& s)
	{
		m_ShapeToUseNext = &s;
	}

	void CreateShapes(const std::vector<Face*>& faces, std::vector<Tshape*>& newShapes, FaceRelationshipWithOtherShape relationship)
	{
		m_CurrRelationship = relationship;
		auto root = FindNextRootFace(faces);

		while (root)
		{
			newShapes.emplace_back(&CreateShape(*root));
			root = FindNextRootFace(faces);
		}
	}

	const MapToFaceRelationship& GetMapToFaceRelationship() const
	{
		return m_Map;
	}

private:
	Tshape * m_ShapeToUseNext = nullptr;

	std::stack<Face*> m_FaceStack;
	MapToFaceRelationship m_Map;
	FaceRelationshipWithOtherShape m_CurrRelationship;
};
