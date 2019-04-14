#pragma once
#include <vector>
#include <stack>
#include "Shape.h"
#include "CompoundShape.h"
#include "DynamicArray.h"
#include "ShapeChunkTaker.h"
#include "CollisionData.h"

template<class Tshape>
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

	Tshape& CreateNewCompoundShape(Tshape& newShape, Shape& rootSubShape, const Transform& transform)
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

		newShape.GetTransform().SetEqualTo(transform);
		return newShape;
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
	void Disconnect(Shape& toDiscon, Tshape& owner, std::vector<Tshape*>& results)
	{
		Shape::ResetNextHashCounter();

		toDiscon.DisconnectLinks();

		m_SubShapes.clear();
		auto& ss = owner.GetSubShapes();
		m_SubShapes.insert(m_SubShapes.begin(), ss.begin(), ss.end());

		m_BeenVisited.Zero();

		auto& t = owner.GetTransform();

		results.emplace_back(&CreateNewCompoundShape(owner, *FindUnVisitedSubShape(), t));

		auto root = FindUnVisitedSubShape();
		while (root)
		{
			results.emplace_back(&CreateNewCompoundShape(*(new Tshape()), *root, t));
			root = FindUnVisitedSubShape();
		}

		Shape::ResetHashes(m_SubShapes);
	}

private:
	std::vector<Shape*> m_SubShapes;
	DynamicArray<int> m_BeenVisited;
	std::stack<Shape*> m_ShapeStack;
};

template<class Tshape>
class ShapeDestructor
{
private:
	float ApproxDistanceFromShape(const Shape& s, const Vector3& p) const
	{
		auto maxDist = MathU::NegativeInfinity;

		for (auto f : s.GetFaces())
		{
			auto dist = Vector3::Dot(p - f->GetPlaneP0(), f->GetNormal());
			if (dist > maxDist)
				maxDist = dist;
		}
		return maxDist;
	}

	Shape & FindSubShapeToDisconnect(const CompoundShape& shape, const Impulse& cause) const
	{
		auto p = shape.GetTransform().ToLocalPosition(cause.WorldImpulsePoint);
		auto minDist = MathU::Infinity;
		Shape* toDicon = nullptr;

		for (auto ss : shape.GetSubShapes())
		{
			auto dist = ApproxDistanceFromShape(*ss, p);
			if (dist < minDist)
			{
				minDist = dist;
				toDicon = ss;
			}
		}
		return *toDicon;
	}

	Tshape& FindAndRemoveShapeToChunk(std::vector<Tshape*>& disconResults, const Shape& toDiscon) const
	{
		for (auto it = disconResults.begin(); it != disconResults.end(); it++)
		{
			if ((*it)->GetSubShapes()[0] == &toDiscon)
			{
				auto& res = **it;
				disconResults.erase(it);
				return res;
			}
		}

		assert(false);
		return *disconResults[0];
	}

public:
	void Destruct(Tshape& shape, const Impulse& cause, std::vector<Tshape*>& results)
	{
		auto& toDiscon = FindSubShapeToDisconnect(shape, cause);
		m_Disconnector.Disconnect(toDiscon, shape, results);

		auto& toChunk = FindAndRemoveShapeToChunk(results, toDiscon);

		auto refTran = shape.GetTransform();
		for (auto disconShape : results)
			disconShape->InitMassProperties(refTran);

		m_ChunkTaker.Chunk(toChunk, results);
	}

private:
	ShapeDisconnector<Tshape> m_Disconnector;
	ShapeChunkTaker<Tshape> m_ChunkTaker;
};