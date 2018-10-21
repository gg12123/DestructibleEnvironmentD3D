#pragma once
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"
#include "Matrix.h"

class CutShapeFaceCreator
{
public:
	template<class... Ts>
	void SetPoints(Ts&... points)
	{
		m_Points = std::vector<Vector3>{ ...points };
	}

	template<class... Ts>
	void SetSharedPointIndexes(Ts... indexes)
	{
		m_SharedPointIndexes = std::vector<int>{ ...indexes };
	}

	void OnPointsSet()
	{
		assert(m_SharedPointIndexes.size() == m_Points.size());

		auto& p0 = m_Points[0];
		auto& p1 = m_Points[1];
		auto& p2 = m_Points[2];

		m_Normal = Vector3::Cross(p0 - p1, p2 - p1).Normalized();
	}

	Face& Create(const Matrix4& M)
	{
		auto& f = *(new Face()); // pool

		f.StartAddingCenteredPoints(m_Normal, M * m_Points[0], m_SharedPointIndexes[0]);

		for (auto i = 1U; i < m_Points.size(); i++)
			f.AddCenteredPoint(M * m_Points[i], m_SharedPointIndexes[i]);

		return f;
	}

private:
	std::vector<Vector3> m_Points;
	std::vector<int> m_SharedPointIndexes;
	Vector3 m_Normal;
};

class CutShapeCreator
{
public:
	CutShapeCreator()
	{
		auto s = Vector3(1.0f, 1.0f, 1.0f);

		auto P0 = Vector3(s.x, s.y, s.z);
		auto P1 = Vector3(-s.x, s.y, s.z);
		auto P2 = Vector3(-s.x, s.y, -s.z);
		auto P3 = Vector3(s.x, s.y, -s.z);

		auto P4 = Vector3(s.x, -s.y, s.z);
		auto P5 = Vector3(-s.x, -s.y, s.z);
		auto P6 = Vector3(-s.x, -s.y, -s.z);
		auto P7 = Vector3(s.x, -s.y, -s.z);

		m_FaceCreators[0].SetPoints(P1, P0, P3, P2);
		m_FaceCreators[0].SetSharedPointIndexes(1, 0, 3, 2);

		m_FaceCreators[1].SetPoints(P0, P4, P7, P3);
		m_FaceCreators[1].SetSharedPointIndexes(0, 4, 7, 3);

		m_FaceCreators[2].SetPoints(P3, P7, P6, P2);
		m_FaceCreators[3].SetPoints(P2, P6, P5, P1);
		m_FaceCreators[4].SetPoints(P1, P5, P4, P0);
		m_FaceCreators[5].SetPoints(P4, P5, P6, P7);
	}

	// split point and normal must be in the ref transforms space
	Shape & Create(Transform& refTransform, const Vector3& splitPoint, const Vector3& splitNormal)
	{
		auto& shapeTran = m_Shape->GetTransform();
		shapeTran.SetEqualTo(refTransform);

		// now position all the faces with centre and rotation defined by split point and normal

		return *m_Shape;
	}

private:
	Shape * m_Shape;

	std::vector<CutShapeFaceCreator> m_FaceCreators;
};