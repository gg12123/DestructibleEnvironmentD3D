#pragma once
#include "Vector3.h"

class Face;
class ShapeEdge;

class EdgeFaceIntersection
{
public:
	Face & GetFace() const
	{
		return *m_FacePierced;
	}

	ShapeEdge& GetEdge() const
	{
		return *m_PiercingEdge;
	}

	const auto& GetIntPoint() const
	{
		return m_IntPoint;
	}

private:
	Face * m_FacePierced;
	ShapeEdge* m_PiercingEdge;
	Vector3 m_IntPoint;
};