#pragma once
#include <vector>
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"
#include "Matrix.h"
#include "FaceLinker.h"

class ShapeFaceCreator
{
public:
	ShapeFaceCreator(const std::vector<Vector3>& points, const std::vector<int>& indexes)
	{
		InitFace(m_BaseFace, points, indexes, CalculateNormal(points), Matrix4::Indentity());
	}

	Face& GetBaseFace()
	{
		return m_BaseFace;
	}

	Face& Create(const Matrix4& M, std::vector<Face*>& toNewFace)
	{
		m_JustCreated = new Face();
		InitFace(*m_JustCreated, m_BaseFace.GetCachedPoints(), m_BaseFace.GetSharedPoints(), m_BaseFace.GetNormal(), Matrix4::Indentity());

		toNewFace[m_BaseFace.GetIdForSplitter()] = m_JustCreated;

		return *m_JustCreated;
	}

	void LinkJustCreated(const std::vector<Face*>& toNewFace)
	{
		auto& basesLinks = m_BaseFace.GetLinkedFaces();
		for (auto i = 0U; i < basesLinks.NumRecycled(); i++)
		{
			auto& edgeLinks = basesLinks.At(i);
			for (auto it2 = edgeLinks.begin(); it2 != edgeLinks.end(); it2++)
			{
				auto& linkedNeighbour = *it2;
				auto newlyCreatedNeighbour = toNewFace[linkedNeighbour.GetNeighbour().GetIdForSplitter()];
				m_JustCreated->AddLink(i, *newlyCreatedNeighbour, linkedNeighbour.GetEdgeOnNeighbour());
			}
		}
	}

private:
	Vector3 CalculateNormal(const std::vector<Vector3>& points)
	{
		auto& p0 = points[0];
		auto& p1 = points[1];
		auto& p2 = points[2];

		return Vector3::Cross(p0 - p1, p2 - p1).Normalized();
	}

	void InitFace(Face& face, const std::vector<Vector3>& points, const std::vector<int>& indexes, const Vector3& normal, const Matrix4& M)
	{
		face.StartAddingCenteredPoints(normal, M * points[0], indexes[0]);

		for (auto i = 1U; i < points.size(); i++)
			face.AddCenteredPoint(M * points[i], indexes[i]);
	}

	Face m_BaseFace;
	Face* m_JustCreated = nullptr;
};

class CubeFacesCreator
{
public:
	CubeFacesCreator()
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

		m_FaceCreators.emplace_back(ShapeFaceCreator({ P1, P0, P3, P2 }, { 1, 0, 3, 2 }));
		m_FaceCreators.emplace_back(ShapeFaceCreator({ P0, P4, P7, P3 }, { 0, 4, 7, 3 }));
		m_FaceCreators.emplace_back(ShapeFaceCreator({ P3, P7, P6, P2 }, { 3, 7, 6, 2 }));
		m_FaceCreators.emplace_back(ShapeFaceCreator({ P2, P6, P5, P1 }, { 2, 6, 5, 1 }));
		m_FaceCreators.emplace_back(ShapeFaceCreator({ P1, P5, P4, P0 }, { 1, 5, 4, 0 }));
		m_FaceCreators.emplace_back(ShapeFaceCreator({ P4, P5, P6, P7 }, { 4, 5, 6, 7 }));

		LinkBaseFaces();

		for (auto i = 0U; i < 6; i++)
			m_ToNewFaces.emplace_back(nullptr);

		m_SharedPoints.emplace_back(P0);
		m_SharedPoints.emplace_back(P1);
		m_SharedPoints.emplace_back(P2);
		m_SharedPoints.emplace_back(P3);
		m_SharedPoints.emplace_back(P4);
		m_SharedPoints.emplace_back(P5);
		m_SharedPoints.emplace_back(P6);
		m_SharedPoints.emplace_back(P7);
	}

	// split point and normal must be in the ref transforms space
	void CreateFaces(Shape& shape, const Matrix4& M)
	{
		shape.ClearFaces();

		for (auto it = m_FaceCreators.begin(); it != m_FaceCreators.end(); it++)
			shape.AddFace((*it).Create(M, m_ToNewFaces));

		for (auto it = m_FaceCreators.begin(); it != m_FaceCreators.end(); it++)
			(*it).LinkJustCreated(m_ToNewFaces);

		auto& faces = shape.GetFaces();
	}

	Vector3 GetSharedPoint(int index, const Matrix4& M)
	{
		return M * m_SharedPoints[index];
	}

private:
	void LinkBaseFaces()
	{
		std::vector<Face*> basesFaces;
		std::vector<PerFaceSplitData> splitData; // TODO - decouple this from splitting

		basesFaces.reserve(6);
		splitData.reserve(6);

		for (auto i = 0U; i < m_FaceCreators.size(); i++)
		{
			auto& baseFace = m_FaceCreators[i].GetBaseFace();

			baseFace.SetIdForSplitter(i);
			basesFaces.emplace_back(&baseFace);
			splitData.emplace_back(PerFaceSplitData(FaceRelationshipWithOtherShape::InIntersection)); // in or out relationship is ok - maybe not unkown though
		}

		FaceLinker linker;
		linker.SetPerFaceData(splitData);
		linker.Link(basesFaces, basesFaces);
	}

	std::vector<ShapeFaceCreator> m_FaceCreators;
	std::vector<Face*> m_ToNewFaces;
	std::vector<Vector3> m_SharedPoints;
};
