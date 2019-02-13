#pragma once
#include <vector>
#include "Transform.h"

class Shape;

class CompoundShape
{
public:
	const auto& GetSubShapes() const
	{
		return m_SubShapes;
	}

	void AddSubShape(Shape& s)
	{
		m_SubShapes.emplace_back(&s);
		s.SetOwner(*this);
	}

	void ClearSubShapes()
	{
		m_SubShapes.clear();
	}

private:
	std::vector<Shape*> m_SubShapes;
};
