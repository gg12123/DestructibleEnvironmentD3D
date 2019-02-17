#include "pch.h"
#include "CompoundShape.h"
#include "Shape.h"
#include "ShapePoint.h"

void CompoundShape::AddSubShape(Shape& s)
{
	m_SubShapes.emplace_back(&s);
	s.SetOwner(*this);
}

bool CompoundShape::IntersectsRay(const Ray& worldRay, Vector3& intPoint)
{
	auto localRay = Ray(m_Transform.ToLocalPosition(worldRay.GetOrigin()),
		m_Transform.ToLocalDirection(worldRay.GetDirection()));

	Vector3 p;
	auto closestDist = MathU::Infinity;
	auto hit = false;

	for (auto s : m_SubShapes)
	{
		if (s->IntersectsRay(localRay, p))
		{
			auto dist = (p - localRay.GetOrigin()).MagnitudeSqr();
			if (dist < closestDist)
			{
				closestDist = dist;
				intPoint = p;
				hit = true;
			}
		}
	}

	if (hit)
		intPoint = m_Transform.ToWorldPosition(intPoint);

	return hit;
}

Vector3 CompoundShape::CalcuateCentre() const
{
	auto c = Vector3::Zero();
	auto count = 0u;

	for (auto s : m_SubShapes)
	{
		s->CalculateCentre();
		c += s->GetCentre();
	}
	return c / static_cast<float>(m_SubShapes.size());
}

void CompoundShape::CentreAndCache(const Vector3& centre)
{
	for (auto s : m_SubShapes)
		s->CentreAndCache(centre);
}