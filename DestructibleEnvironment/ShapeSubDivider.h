#pragma once
#include "Shape.h"
#include "ShapeSplitter.h"

template<class Tshape>
class ShapeSubDivider
{
public:
	void DivideShape(int numDivisions, std::vector<Tshape*>& newShapes, Tshape& original)
	{
		m_ShapesToDivide.clear();
		newShapes.clear();

		m_ShapesToDivide.emplace_back(&original);

		for (auto i = 0; i < numDivisions; i++)
		{
			for (auto s : m_ShapesToDivide)
			{
				if (!m_Splitter.Split(s->GetTransform().GetPosition(), *s, newShapes))
					return;
			}

			m_ShapesToDivide.swap(newShapes);
		}

		m_ShapesToDivide.swap(newShapes);
	}

private:
	ShapeSplitter<Tshape> m_Splitter;
	std::vector<Tshape*> m_ShapesToDivide;
};
