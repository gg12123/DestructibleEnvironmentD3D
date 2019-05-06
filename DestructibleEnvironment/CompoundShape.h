#pragma once
#include <vector>
#include "Transform.h"
#include "ShapeElementPool.h"
#include "Ray.h"

class Shape;
class ShapeProxy;
class PhysicsObject;

class CompoundShape : public AlignedObject16
{
public:
	CompoundShape()
	{
		m_NextId++;
		m_Id = m_NextId;
	}

	virtual ~CompoundShape()
	{
		for (auto s : m_SubShapes)
			ShapePool::Return(*s);
	}

	int GetCompoundShapeId() const
	{
		return m_Id;
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

	bool IsDirty()
	{
		return m_Dirty;
	}

	void ClearDirty()
	{
		m_Dirty = false;
	}

	const auto& GetTransform() const
	{
		return m_Transform;
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

	virtual PhysicsObject* ToPhysicsObject()
	{
		assert(false);
		return nullptr;
	}

	bool IsAwake() const
	{
		return m_IsAwake;
	}

	bool IsStatic() const
	{
		return m_IsStatic;
	}

protected:
	void UpdateSubShapesWorldAABBs() const;

	void SetAwake(bool val)
	{
		m_IsAwake = val;
	}

	void SetStatic(bool val)
	{
		m_IsStatic = val;
	}

	// All sub-shapes and the input centre are expressed in the ref transforms space
	void CentreAndCache(const Transform& refTran, const Vector3& c)
	{
		CentreAndCache(c);

		m_Transform.SetPosition(refTran.ToWorldPosition(c));
		m_Transform.SetRotation(refTran.GetRotation());

		UpdateSubShapesWorldAABBs();

		SetDirty();
	}

private:
	void CentreAndCache(const Vector3& centre);

	void SetDirty()
	{
		m_Dirty = true;
	}

	std::vector<Shape*> m_SubShapes;
	Transform m_Transform;
	bool m_Dirty = true;
	ShapeProxy* m_Proxy;
	bool m_IsAwake;
	bool m_IsStatic;

	static int m_NextId;
	int m_Id;
};
