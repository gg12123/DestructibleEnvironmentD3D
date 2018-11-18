#pragma once
#include "Vector3.h"

class Face;
class ShapeEdge;

class EdgeFaceIntersection
{
public:
	EdgeFaceIntersection()
	{
		m_FacePierced = nullptr;
		m_PiercingEdge = nullptr;
	}

	EdgeFaceIntersection(Face& facePierced, ShapeEdge& piercingEdge)
	{
		m_FacePierced = &facePierced;
		m_PiercingEdge = &piercingEdge;
	}

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

	bool operator ==(const EdgeFaceIntersection& rhs) const
	{
		return m_FacePierced == rhs.m_FacePierced && m_PiercingEdge == rhs.m_PiercingEdge;
	}

	bool operator !=(const EdgeFaceIntersection& rhs) const
	{
		return !(*this == rhs);
	}

private:
	Face * m_FacePierced;
	ShapeEdge* m_PiercingEdge;
	Vector3 m_IntPoint;
};