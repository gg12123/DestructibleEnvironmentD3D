#pragma once
#include "DynamicMesh.h"
#include <vector>

class CompoundShape;

struct SubShapeData
{
	Vector3 Centre;
	Vector3 Size;

	SubShapeData(const Vector3& c, const Vector3& s)
	{
		Centre = c;
		Size = s;
	}
};

struct SubShapeLink
{
	int ShapeAIndex;
	int ShapeBIndex;

	SubShapeLink(int aIndex, int bIndex)
	{
		ShapeAIndex = aIndex;
		ShapeBIndex = bIndex;
	}
};

// TODO - not all shapes need a dynamic mesh so this needs sorting at some point.
// could just specify the required buffer type to the base.
// I think verts could still be inserted using the same map in sync method
// because non dynamic buffers still allow CPU to write.
class ShapeProxy : public DynamicMesh
{
public:
	ShapeProxy()
	{
	}

	// Special constructor used when a shape is added by the physics engine.
	ShapeProxy(CompoundShape& shape);

	void Syncronise();

	const auto& GetSubShapeData() const
	{
		return m_SubShapes;
	}

	const auto& GetSubShapeLinks() const
	{
		return m_SubShapeLinks;
	}

	void AddSubShapeData(const SubShapeData& subShape)
	{
		m_SubShapes.emplace_back(subShape);
	}

	void AddSubShapeLink(const SubShapeLink& link)
	{
		m_SubShapeLinks.emplace_back(link);
	}

protected:
	void Awake() override;

	virtual CompoundShape& RegisterWithPhysics() = 0;
private:
	CompoundShape * m_Shape = nullptr;
	SimdStdVector<SubShapeData> m_SubShapes;
	std::vector<SubShapeLink> m_SubShapeLinks;
};