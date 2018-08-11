#pragma once
#include "DynamicMesh.h"

class Shape;

class ShapeProxy : public DynamicMesh
{
public:
	void Syncronise();

	virtual void FixedUpdate()
	{
		// could call this on the physics thread
		// and use it to add forces to the physics shape (not this one!).
	}

protected:
	void Awake() override;

private:
	Shape * m_Shape = nullptr;
};
