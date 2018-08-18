#pragma once
#include "DynamicMesh.h"

class Shape;

class ShapeProxy : public DynamicMesh
{
public:
	ShapeProxy()
	{
	}

	// special constructor used when a shape is added by the physics engine.
	ShapeProxy(Shape& shape)
	{
		m_Shape = &shape;
	}

	void Syncronise();

	virtual void FixedUpdate()
	{
		// could call this on the physics thread
		// and use it to add forces to the physics shape (not this one!).
	}

	float GetInitialWidth() const
	{
		return m_InitialWidth;
	}

	float GetInitialHeight() const
	{
		return m_InitialWidth;
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

private:
	Shape * m_Shape = nullptr;

	float m_InitialWidth;
	float m_InitialHeight;
};