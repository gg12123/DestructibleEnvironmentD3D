#include "CompoundShape.h"
#include "Shape.h"
#include "ShapePoint.h"

Vector3 CompoundShape::CalcuateCentre()
{
	auto c = Vector3::Zero();
	auto count = 0u;

	for (auto s : m_SubShapes)
	{
		auto& points = s->GetPointObjects();

		for (auto p : points)
			c += p->GetPoint();

		count += points.size();
	}
	return c / static_cast<float>(count);
}

void CompoundShape::CentreAndCache(const Vector3& centre)
{
	for (auto s : m_SubShapes)
		s->CentreAndCache(centre);
}