#pragma once
#include <vector>
#include <array>
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"
#include "Matrix.h"
#include "TwoDArray.h"

class CubeFacesCreator
{
private:
	static constexpr int CutShapeNumPoints = 8;
	static constexpr int FaceNumPoints = 3;

	void CreatePoints(const Matrix4& M)
	{
		auto s = Vector3(1.0f, 1.0f, 1.0f);

		// TODO - get from pool
		m_Points[0] = new ShapePoint(M * Vector3(s.x, s.y, s.z));
		m_Points[1] = new ShapePoint(M * Vector3(-s.x, s.y, s.z));
		m_Points[2] = new ShapePoint(M * Vector3(-s.x, s.y, -s.z));
		m_Points[3] = new ShapePoint(M * Vector3(s.x, s.y, -s.z));

		m_Points[4] = new ShapePoint(M * Vector3(s.x, -s.y, s.z));
		m_Points[5] = new ShapePoint(M * Vector3(-s.x, -s.y, s.z));
		m_Points[6] = new ShapePoint(M * Vector3(-s.x, -s.y, -s.z));
		m_Points[7] = new ShapePoint(M * Vector3(s.x, -s.y, -s.z));
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

		CreateEdge(1, 3);
		CreateEdge(7, 0);
		CreateEdge(6, 3);
		CreateEdge(5, 2);
		CreateEdge(4, 1);
		CreateEdge(6, 4);
	}

	Vector3 DirToNext(int p, int pNext)
	{
		return (m_Points[pNext]->GetPoint() - m_Points[p]->GetPoint()).Normalized();
	}

	void CreateEdge(int p, int pNext)
	{
		// TODO - get from pool
		auto edge = *(new ShapeEdge(*m_Points[p], *m_Points[pNext], DirToNext(p, pNext)));

		m_Edges.Get(p, pNext) = &edge;
		m_Edges.Get(pNext, p) = &edge;
	}

	ShapeEdge& EdgeToNext(int p, int pNext)
	{
		return *m_Edges.Get(p, pNext);
	}

	template<int numPoints>
	Face& CreateFace(const std::array<int, numPoints>& pointIndexes)
	{
		// TODO - get from pool
		auto& f = *(new Face());

		for (auto i = 0U; i < numPoints; i++)
		{
			auto nextI = (i + 1) % numPoints;

			f.AddPoint(*m_Points[i], DirToNext(i, nextI), EdgeToNext(i, nextI));
		}
		return f;
	}

	void CreateFaces(Shape& shape)
	{
		shape.AddFace(CreateFace(m_Face0));
		shape.AddFace(CreateFace(m_Face1));
		shape.AddFace(CreateFace(m_Face2));
		shape.AddFace(CreateFace(m_Face3));
		shape.AddFace(CreateFace(m_Face4));
		shape.AddFace(CreateFace(m_Face5));
		shape.AddFace(CreateFace(m_Face6));
		shape.AddFace(CreateFace(m_Face7));
		shape.AddFace(CreateFace(m_Face8));
		shape.AddFace(CreateFace(m_Face9));
		shape.AddFace(CreateFace(m_Face10));
		shape.AddFace(CreateFace(m_Face11));
	}

public:
	CubeFacesCreator()
	{
		m_Face0 = { 1, 0, 3 };
		m_Face1 = { 3, 2, 1 };

		m_Face2 = { 0, 4, 7 };
		m_Face3 = { 7, 3, 0 };

		m_Face4 = { 3, 7, 6 };
		m_Face5 = { 6, 2, 3 };

		m_Face6 = { 2, 6, 5 };
		m_Face7 = { 5, 1, 2 };

		m_Face8 = { 1, 5, 4 };
		m_Face9 = { 4, 0, 1 };

		m_Face10 = { 4, 5, 6 };
		m_Face11 = { 6, 7, 4 };
	}

	void CreateFaces(Shape& shape, const Matrix4& M)
	{
		CreatePoints(M);
		CreateEdges();
		CreateFaces(shape);
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
	std::array<int, FaceNumPoints> m_Face6;
	std::array<int, FaceNumPoints> m_Face7;
	std::array<int, FaceNumPoints> m_Face8;
	std::array<int, FaceNumPoints> m_Face9;
	std::array<int, FaceNumPoints> m_Face10;
	std::array<int, FaceNumPoints> m_Face11;
};
