#pragma once
#include <vector>
#include <stack>
#include "Shape.h"
#include "CompoundShape.h"
#include "DynamicArray.h"

class ShapeDisconnector
{
private:
	bool IsVisited(Shape& s)
	{
		s.TryAssignHash();
		return m_BeenVisited[s.GetHash()];
	}

	void MarkVisited(Shape& s)
	{
		s.TryAssignHash();
		m_BeenVisited[s.GetHash()] = 1;
	}

	void CreateNewCompoundShape(CompoundShape& newShape, Shape& rootSubShape)
	{
		newShape.ClearSubShapes();

		m_ShapeStack.push(&rootSubShape);
		while (!m_ShapeStack.empty())
		{
			auto next = m_ShapeStack.top();
			m_ShapeStack.pop();

			if (IsVisited(*next))
				continue;

			newShape.AddSubShape(*next);
			MarkVisited(*next);

			for (auto link : next->GetLinkedShapes())
			{
				if (!IsVisited(*link))
					m_ShapeStack.push(link);
			}
		}
	}

	Shape* FindUnVisitedSubShape()
	{
		for (auto s : m_SubShapes)
		{
			if (!IsVisited(*s))
				return s;
		}
		return nullptr;
	}

public:
	void Disconnect(Shape& toDiscon, CompoundShape& owner, std::vector<CompoundShape*>& results)
	{
		Shape::ResetNextHashCounter();

		toDiscon.DisconnectLinks();

		m_SubShapes.clear();
		auto& ss = owner.GetSubShapes();
		m_SubShapes.insert(m_SubShapes.begin(), ss.begin(), ss.end());

		m_BeenVisited.Zero();

		CreateNewCompoundShape(owner, *FindUnVisitedSubShape());

		auto root = FindUnVisitedSubShape();
		while (root)
		{
			CreateNewCompoundShape(*(new CompoundShape()), *root);
			root = FindUnVisitedSubShape();
		}

		Shape::ResetHashes(m_SubShapes);
	}

private:
	std::vector<Shape*> m_SubShapes;
	DynamicArray<int> m_BeenVisited;
	std::stack<Shape*> m_ShapeStack;
};