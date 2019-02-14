#pragma once
#include "Shape.h"
#include "ShapeSplitter.h"
#include "CompoundShape.h"

template<class Tshape>
class ShapeSubDivider
{
private:
	Vector3 CalculateCentre(Shape& s)
	{
		auto c = Vector3::Zero();

		for (auto p : s.GetPointObjects())
			c += p->GetPoint();

		return c / static_cast<float>(s.GetPointObjects().size());
	}

	Plane CalculateSplitPlane(Shape& s)
	{
		auto a = Random::Range(0.01f, 1.0f);
		auto b = Random::Range(0.01f, 1.0f);
		auto c = Random::Range(0.01f, 1.0f);

		a *= Random::Range(0.0f, 1.0f) > 0.5f ? -1.0f : 1.0f;
		b *= Random::Range(0.0f, 1.0f) > 0.5f ? -1.0f : 1.0f;
		c *= Random::Range(0.0f, 1.0f) > 0.5f ? -1.0f : 1.0f;

		return Plane(Vector3(a, b, c).Normalized(), CalculateCentre(s));

	}

public:
	void DivideShape(int numDivisions, std::vector<Tshape*>& newShapes, Tshape& original)
	{
		auto refTran = original.GetTransform();

		m_ShapesToDivide.clear();
		m_ShapesToDivideNext.clear();

		for (auto s : original.GetSubShapes())
			m_ShapesToDivide.emplace_back(s);

		for (auto i = 0; i < numDivisions; i++)
		{
			for (auto s : m_ShapesToDivide)
			{
				assert(m_Splitter.Split(CalculateSplitPlane(*s), *s, m_ShapesToDivideNext, m_ShapesToDivideNext));
			}

			m_ShapesToDivide.swap(m_ShapesToDivideNext);
			m_ShapesToDivideNext.clear();
		}

		original.ClearSubShapes();
		original.AddSubShape(*m_ShapesToDivide[0]);
		original.CentreAndCache(refTran);
		newShapes.emplace_back(&original);

		for (auto i = 1u; i < m_ShapesToDivide.size(); i++)
		{
			auto s = new Tshape();
			s->AddSubShape(*m_ShapesToDivide[i]);
			s->CentreAndCache(refTran);
			newShapes.emplace_back(s);
		}
	}

private:
	ShapeSplitter<Shape> m_Splitter;

	std::vector<Shape*> m_ShapesToDivide;
	std::vector<Shape*> m_ShapesToDivideNext;
};
