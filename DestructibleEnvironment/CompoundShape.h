#pragma once
#include <vector>
#include "Transform.h"
#include "ShapeElementPool.h"
#include "Ray.h"

class Shape;
class ShapeProxy;

class CompoundShape
{
public:
	~CompoundShape()
	{
		// TODO - also need to return all the shapes points edge and faces to the pool
		for (auto s : m_SubShapes)
			ShapePool::Return(*s);
	}

	bool IntersectsRay(const Ray& worldRay, Vector3& intPoint);

	const auto& GetSubShapes() const
	{
		return m_SubShapes;
	}

	void AddSubShape(Shape& s);

	void ClearSubShapes()
	{
		m_SubShapes.clear();
	}

	// All sub-shapes are expressed in the ref transforms space
	void CentreAndCache(Transform& refTran)
	{
		auto c = CalcuateCentre();
		CentreAndCache(c);

		m_Transform.SetPosition(refTran.ToWorldPosition(c));
		m_Transform.SetRotation(refTran.GetRotation());

		SetDirty();
	}

	bool IsDirty()
	{
		return m_Dirty;
	}

	void ClearDirty()
	{
		m_Dirty = false;
	}

	auto& GetTransform()
	{
		return m_Transform;
	}

	auto& GetProxy() const
	{
		return *m_Proxy;
	}

	void SetProxy(ShapeProxy& p)
	{
		m_Proxy = &p;
	}

private:
	Vector3 CalcuateCentre() const;
	void CentreAndCache(const Vector3& centre);

	void SetDirty()
	{
		m_Dirty = true;
	}

	std::vector<Shape*> m_SubShapes;
	Transform m_Transform;
	bool m_Dirty = true;
	ShapeProxy* m_Proxy;
};
