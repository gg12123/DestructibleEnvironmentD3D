#pragma once
#include <vector>
#include "Vector3.h"

class Face;

class ConvexFaceCreator
{
public:
	void AddPoint(const Vector3& p)
	{
		m_Points.emplace_back(p);
	}

	void Restart()
	{
		m_Points.clear();
	}

	void CreateFaces(std::vector<Face*>& faces);
	void CreateUpsideDownFaces(std::vector<Face*>& faces);

	void SetNormal(const Vector3& normal)
	{
		m_Normal = normal;
	}

private:
	std::vector<Vector3> m_Points;
	Vector3 m_Normal;
};
