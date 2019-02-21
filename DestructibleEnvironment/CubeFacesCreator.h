#pragma once
#include <vector>
#include <array>
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"
#include "Matrix.h"
#include "TwoDArray.h"
#include "ShapeElementPool.h"

class CubeFacesCreator
{
private:
	static constexpr int CutShapeNumPoints = 8;
	static constexpr int FaceNumPoints = 4;

	void CreatePoints(const Matrix4& M)
	{
		auto s = Vector3(1.0f, 1.0f, 1.0f);

		m_Points[0] = &PointPool::Take(M * Vector3(s.x, s.y, s.z));
		m_Points[1] = &PointPool::Take(M * Vector3(-s.x, s.y, s.z));
		m_Points[2] = &PointPool::Take(M * Vector3(-s.x, s.y, -s.z));
		m_Points[3] = &PointPool::Take(M * Vector3(s.x, s.y, -s.z));

		m_Points[4] = &PointPool::Take(M * Vector3(s.x, -s.y, s.z));
		m_Points[5] = &PointPool::Take(M * Vector3(-s.x, -s.y, s.z));
		m_Points[6] = &PointPool::Take(M * Vector3(-s.x, -s.y, -s.z));
		m_Points[7] = &PointPool::Take(M * Vector3(s.x, -s.y, -s.z));
	}

	void CreateEdges()
	{
		CreateEdge(0, 1);
		CreateEdge(1, 2);
		CreateEdge(2, 3);
		CreateEdge(3, 0);

		CreateEdge(4, 5);
		CreateEdge(5, 6);
		CreateEdge(6, 7);
		CreateEdge(7, 4);

		CreateEdge(0, 4);
		CreateEdge(3, 7);
		CreateEdge(1, 5);
		CreateEdge(2, 6);

		//CreateEdge(1, 3);
		//CreateEdge(7, 0);
		//CreateEdge(6, 3);
		//CreateEdge(5, 2);
		//CreateEdge(4, 1);
		//CreateEdge(6, 4);
	}

	Vector3 DirToNext(int p, int pNext)
	{
		return (m_Points[pNext]->GetPoint() - m_Points[p]->GetPoint()).Normalized();
	}

	void CreateEdge(int p, int pNext)
	{
		auto& edge = EdgePool::Take(*m_Points[p], *m_Points[pNext]);

		m_Edges.Get(p, pNext) = &edge;
		m_Edges.Get(pNext, p) = &edge;
	}

	ShapeEdge& EdgeToNext(int p, int pNext)
	{
		return *m_Edges.Get(p, pNext);
	}

	template<int numPoints>
	Face& CreateFace(const std::array<int, numPoints>& pointIndexes, const Vector3& normal)
	{
		auto& f = FacePool::Take();

		for (auto i = 0U; i < numPoints; i++)
		{
			auto j = pointIndexes[i];
			auto k = pointIndexes[(i + 1) % numPoints];

			f.AddPoint(*m_Points[j], EdgeToNext(j, k));
		}
		f.SetNormal(normal);

		return f;
	}

	void CreateFaces(Shape& shape, const Quaternion& q)
	{
		auto upRot = q.RotateV(Vector3::Up());
		auto rightRot = q.RotateV(Vector3::Right());
		auto forwardRot = q.RotateV(Vector3::Foward());

		shape.AddFace(CreateFace(m_Face0, upRot));
		//shape.AddFace(CreateFace(m_Face1, upRot));

		shape.AddFace(CreateFace(m_Face1, rightRot));
		//shape.AddFace(CreateFace(m_Face3, rightRot));

		shape.AddFace(CreateFace(m_Face2, -forwardRot));
		//shape.AddFace(CreateFace(m_Face5, -forwardRot));

		shape.AddFace(CreateFace(m_Face3, -rightRot));
		//shape.AddFace(CreateFace(m_Face7, -rightRot));

		shape.AddFace(CreateFace(m_Face4, forwardRot));
		//shape.AddFace(CreateFace(m_Face9, forwardRot));

		shape.AddFace(CreateFace(m_Face5, -upRot));
		//shape.AddFace(CreateFace(m_Face11, -upRot));
	}

public:
	CubeFacesCreator()
	{
		m_Face0 = { 1, 0, 3, 2 };
		//m_Face1 = { 3, 2, 1 };

		m_Face1 = { 0, 4, 7, 3 };
		//m_Face3 = { 7, 3, 0 };

		m_Face2 = { 3, 7, 6, 2 };
		//m_Face5 = { 6, 2, 3 };

		m_Face3 = { 2, 6, 5, 1 };
		//m_Face7 = { 5, 1, 2 };

		m_Face4 = { 1, 5, 4, 0 };
		//m_Face9 = { 4, 0, 1 };

		m_Face5 = { 4, 5, 6, 7 };
		//m_Face11 = { 6, 7, 4 };
	}

	void CreateFaces(Shape& shape, const Matrix4& M, const Quaternion& MsRot)
	{
		CreatePoints(M);
		CreateEdges();
		CreateFaces(shape, MsRot);
	}

private:
	std::array<ShapePoint*, CutShapeNumPoints> m_Points;
	TwoDArray<CutShapeNumPoints, CutShapeNumPoints, ShapeEdge*> m_Edges;

	std::array<int, FaceNumPoints> m_Face0;
	std::array<int, FaceNumPoints> m_Face1;
	std::array<int, FaceNumPoints> m_Face2;
	std::array<int, FaceNumPoints> m_Face3;
	std::array<int, FaceNumPoints> m_Face4;
	std::array<int, FaceNumPoints> m_Face5;
};
