#pragma once
#include "ShapeProxy.h"

class StaticBody;

class StaticShapeProxy : public ShapeProxy
{
public:
	StaticBody & GetStaticBody() const
	{
		return *m_StaticBody;
	}

protected:
	CompoundShape & RegisterWithPhysics() override;

private:
	StaticBody * m_StaticBody;
};