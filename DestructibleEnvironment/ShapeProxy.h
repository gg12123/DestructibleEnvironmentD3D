#pragma once
#include "DynamicMesh.h"

class Shape;

// TODO - not all shapes need a dynamic mesh to this needs sorting at some point.
// could just specify the required buffer type to the base.
// I think verts could still be inserted using the same map in sync method
// because non dynamic buffers still allow CPU to write.
class ShapeProxy : public DynamicMesh
{
public:
	ShapeProxy()
	{
	}

	// special constructor used when a shape is added by the physics engine.
	ShapeProxy(Shape& shape);

	void Syncronise();

	float GetInitialWidth() const
	{
		return m_InitialWidth;
	}

	float GetInitialHeight() const
	{
		return m_InitialHeight;
	}

	void SetInitialWidth(float val)
	{
		m_InitialWidth = val;
	}

	void SetInitialHeight(float val)
	{
		m_InitialHeight = val;
	}

protected:
	void Awake() override;

	virtual Shape& RegisterWithPhysics() = 0;
private:
	Shape * m_Shape = nullptr;

	float m_InitialWidth;
	float m_InitialHeight;
};