#pragma once
#include <array>
#include "Shape.h"
#include "Plane.h"
#include "ShapeSplitter.h"
#include "CompoundShape.h"
#include "Pool.h"

class ShapeChunkTaker
{
private:
	static constexpr int MaxNumSplitPlanes = 5;

	enum class PlaneRelationship
	{
		Above,
		Below
	};

	class ShapeWithPlaneRelationships
	{
	public:
		ShapeWithPlaneRelationships(Shape& s) : m_Shape(&s)
		{
		}

		void CopyRelationships(const std::array<PlaneRelationship, MaxNumSplitPlanes>& relationships)
		{
			std::memcpy(m_Relationships.data(), relationships.data(), MaxNumSplitPlanes * sizeof(PlaneRelationship));
		}

		void SetRelationship(int plane, PlaneRelationship r)
		{
			m_Relationships[plane] = r;
		}

		auto& GetShape() const
		{
			return *m_Shape;
		}

		const auto& GetRelationships() const
		{
			return m_Relationships;
		}

		bool CanLinkAboutPlane(int plane, int numPlanes, const ShapeWithPlaneRelationships& other) const
		{
			auto& myR = m_Relationships;
			auto& otherR = other.m_Relationships;

			if (myR[plane] == otherR[plane])
				return false;

			for (int i = 0; i < numPlanes; i++)
			{
				if (i == plane)
					continue;

				if (myR[i] != otherR[i])
					return false;
			}
			return true;
		}

		bool IsBelowOnly(int numPlanes) const
		{
			for (int i = 0; i < numPlanes; i++)
			{
				if (m_Relationships[i] == PlaneRelationship::Above)
					return false;
			}
			return true;
		}

	private:
		Shape * m_Shape;
		std::array<PlaneRelationship, MaxNumSplitPlanes> m_Relationships;
	};

	void CalculateSplitPlanes(const Plane& chunkPlane)
	{

	}

	void HandleNewSubShapes(const ShapeWithPlaneRelationships& splitFrom,
		PlaneRelationship r,
		int planeId,
		const std::vector<Shape*>& newShapes)
	{
		for (auto s : newShapes)
		{
			auto sWithR = ShapeWithPlaneRelationships(*s);
			sWithR.CopyRelationships(splitFrom.GetRelationships());
			sWithR.SetRelationship(planeId, r);
			m_BufferNext.emplace_back(sWithR);
		}
	}

	void CalculateSubShapes(CompoundShape& origShape)
	{
		for (auto s : origShape.GetSubShapes())
			m_BufferActive.emplace_back(ShapeWithPlaneRelationships(*s));

		for (auto i = 0u; i < m_SplitPlanes.size(); i++)
		{
			auto& plane = m_SplitPlanes[i];
			for (auto& subShape : m_BufferActive)
			{
				m_NewShapesAbove.clear();
				m_NewShapesBelow.clear();
				m_Splitter.Split(plane, subShape.GetShape(), m_NewShapesAbove, m_NewShapesBelow);

				HandleNewSubShapes(subShape, PlaneRelationship::Above, i, m_NewShapesAbove);
				HandleNewSubShapes(subShape, PlaneRelationship::Below, i, m_NewShapesBelow);
			}

			m_BufferActive.swap(m_BufferNext);
		}
	}

	bool ShouldBeLinked(const ShapeWithPlaneRelationships& s1, const ShapeWithPlaneRelationships& s2) const
	{
		for (auto i = 0u; i < m_SplitPlanes.size(); i++)
		{
			if (s1.CanLinkAboutPlane(i, m_SplitPlanes.size(), s2))
				return true;
		}
		return false;
	}

	void HandleBelowOnlyShapes()
	{
		for (auto it = m_BufferActive.begin(); it != m_BufferActive.end(); it++)
		{
			if (it->IsBelowOnly(m_SplitPlanes.size()))
			{
				auto& cs = TakeCompoundShape();
				cs.AddSubShape(it->GetShape());
				m_BufferActive.erase(it);
				it--;
			}
		}
	}

	CompoundShape& TakeCompoundShape()
	{
		if (m_CsToUseNext)
		{
			m_CsToUseNext->ClearSubShapes();
			auto x = m_CsToUseNext;
			m_CsToUseNext = nullptr;
			return *x;
		}

		auto cs = m_ShapePool.GetObject();
		cs->ClearSubShapes();
		return *cs;
	}

	void LinkShapes(Shape& s1, Shape& s2)
	{
		if (!s1.HasOwner() && !s2.HasOwner())
		{
			auto& cs = TakeCompoundShape();
			cs.AddSubShape(s1);
			cs.AddSubShape(s2);
		}
		else if (s1.HasOwner() && s2.HasOwner())
		{
			auto& csToKeep = s1.GetOwner();
			auto& csToLose = s2.GetOwner();

			for (auto s : csToLose.GetSubShapes())
				csToKeep.AddSubShape(*s);

			m_ShapePool.Return(&csToLose);
		}
		else if (s1.HasOwner())
		{
			s1.GetOwner().AddSubShape(s2);
		}
		else
		{
			s2.GetOwner().AddSubShape(s1);
		}
	}

	void FormCompoundShapes(std::vector<CompoundShape*>& newShapes)
	{
		// This must be done before the linking so that the original input compound shape
		// is garunteed to be re-used.
		// If it doesnt get re-used until the linking, it may get put back in the pool.
		HandleBelowOnlyShapes();

		for (auto i = 0u; i < m_BufferActive.size(); i++)
		{
			auto& shape = m_BufferActive[i];
			for (auto j = 0u; j < m_BufferActive.size(); j++)
			{
				auto& otherShape = m_BufferActive[j];

				if (ShouldBeLinked(shape, otherShape))
					LinkShapes(shape.GetShape(), otherShape.GetShape());
			}
		}

		for (auto& s : m_BufferActive)
		{
			auto& owner = s.GetShape().GetOwner();
			if (!CollectionU::Contains(newShapes, &owner))
				newShapes.emplace_back(&owner);
		}
	}

public:
	ShapeChunkTaker() : m_ShapePool([]() { return new CompoundShape(); } , 5)
	{
	}

	void TakeChunk(CompoundShape& origShape, const Plane& chunkPlane, std::vector<CompoundShape*>& newShapes)
	{
		// Must make a copy
		auto refTran = origShape.GetTransform();

		CalculateSplitPlanes(chunkPlane);
		CalculateSubShapes(origShape);

		m_CsToUseNext = &origShape;
		FormCompoundShapes(newShapes);

		for (auto s : newShapes)
			s->CentreAndCache(refTran);
	}

private:
	std::vector<ShapeWithPlaneRelationships> m_BufferActive;
	std::vector<ShapeWithPlaneRelationships> m_BufferNext;

	std::vector<Shape*> m_NewShapesAbove;
	std::vector<Shape*> m_NewShapesBelow;

	std::vector<Plane> m_SplitPlanes;

	ShapeSplitter m_Splitter;

	Pool<CompoundShape*> m_ShapePool;
	CompoundShape* m_CsToUseNext;
};