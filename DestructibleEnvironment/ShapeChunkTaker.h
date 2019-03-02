#pragma once
#include <array>
#include "Shape.h"
#include "Plane.h"
#include "ShapeSplitter.h"
#include "CompoundShape.h"
#include "Pool.h"

template<class Tshape>
class ShapeChunkTaker
{
private:
	static constexpr int TetraNumVerts = 4;

	class TetraVerts
	{
	public:
		std::array<Vector3, TetraNumVerts> Verts;

		TetraVerts()
		{
			m_OrigVerts[0] = Vector3(std::sqrtf(8.0f / 9.0f), 0.0f, -1.0f / 3.0f);
			m_OrigVerts[1] = Vector3(-std::sqrtf(2.0f / 9.0f), std::sqrtf(2.0f / 3.0f), -1.0f / 3.0f);
			m_OrigVerts[2] = Vector3(-std::sqrtf(2.0f / 9.0f), -std::sqrtf(2.0f / 3.0f), -1.0f / 3.0f);
			m_OrigVerts[3] = Vector3(0.0f, 0.0f, 1.0f);
		}

		void ApplyRandomTransform()
		{
			auto r = Quaternion(Random::Range(0.1f, 1.0f), Random::Range(0.1f, 1.0f), Random::Range(0.1f, 1.0f), Random::Range(0.1f, 1.0f));
			r.Normalize();

			for (auto i = 0u; i < m_OrigVerts.size(); i++)
				Verts[i] = r.RotateV(m_OrigVerts[i]);
		}

	private:
		std::array<Vector3, TetraNumVerts> m_OrigVerts;
	};

	Shape& Duplicate(const Shape& s)
	{
		return s.Duplicate();
	}

	void CalculatePlanes(const Vector3& t0, const Vector3& t1, const Vector3& t2, std::array<Plane, 3>& planes)
	{
		auto c = (t0 + t1 + t2) / 2.0f;

		auto n0 = Vector3::Cross(t0, t1).InDirectionOf(t0 - c).Normalized();
		auto n1 = Vector3::Cross(t1, t2).InDirectionOf(t1 - c).Normalized();
		auto n2 = Vector3::Cross(t2, t0).InDirectionOf(t2 - c).Normalized();

		planes[0] = Plane(n0, t0);
		planes[1] = Plane(n1, t1);
		planes[2] = Plane(n2, t2);
	}

	Shape& TakeChunk(Shape& toChunk, const std::array<Plane, 3>& planes)
	{
		Shape* above = nullptr;
		Shape* below = &toChunk;

		for (auto& sp : planes)
		{
			assert(m_Splitter.Split(sp, *below, &above, &below));

			ShapePool::Return(*above);
			below->ResetBeenCollectedFlag();
		}
		return *below;
	}

	Tshape& MakeShape(Tshape& shape, Shape& geometry, Transform& refTran)
	{
		shape.ClearSubShapes();
		shape.AddSubShape(geometry);
		shape.CentreAndCache(refTran);
		return shape;
	}

public:
	void Chunk(Tshape& toChunk, std::vector<Tshape*>& chunks)
	{
		assert(toChunk.GetSubShapes().size() == 1u);

		auto& shape = *toChunk.GetSubShapes()[0];
		auto refTran = toChunk.GetTransform();

		m_Tetra.ApplyRandomTransform();
		m_NewShapes.clear();

		auto& verts = m_Tetra.Verts;
		std::array<Plane, 3> planes;

		CalculatePlanes(verts[0], verts[1], verts[2], planes);
		m_NewShapes.emplace_back(&TakeChunk(Duplicate(shape), planes));

		CalculatePlanes(verts[1], verts[2], verts[3], planes);
		m_NewShapes.emplace_back(&TakeChunk(Duplicate(shape), planes));

		CalculatePlanes(verts[0], verts[1], verts[3], planes);
		m_NewShapes.emplace_back(&TakeChunk(Duplicate(shape), planes));

		CalculatePlanes(verts[0], verts[2], verts[3], planes);
		m_NewShapes.emplace_back(&TakeChunk(shape, planes)); // No need to duplicate on the final chunk

		chunks.emplace_back(&MakeShape(toChunk, *m_NewShapes[0], refTran));

		for (auto i = 1u; i < m_NewShapes.size(); i++)
			chunks.emplace_back(&MakeShape((*new Tshape()), *m_NewShapes[i], refTran));
	}

private:
	ShapeSplitter m_Splitter;
	TetraVerts m_Tetra;
	std::vector<Shape*> m_NewShapes;
};