#pragma once
#include <vector>
#include <memory>
#include <algorithm>
#include "EdgeFaceIntersection.h"
#include "PoolOfRecyclables.h"
#include "FacesCutPath.h"
#include "CutPathElement.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "MapToFacesCutPath.h"

class FacesCutPaths
{
public:
	FacesCutPaths(MapToFacesCutPath& map)
	{
		m_MapToFCP = &map;
	}

	void Init(Face& face)
	{
		m_Paths.clear();
		m_Indexs.clear();
		m_Face = &face;
	}

	const auto& GetPaths() const
	{
		return m_Paths;
	}

	Face& GetFace() const
	{
		return *m_Face;
	}

	void AddFirst(int index)
	{
		m_Indexs.emplace_back(index);
	}

	void AddFinal(int index, PoolOfRecyclables<FacesCutPath>& cutPaths, const std::vector<CutPathElement>& cp)
	{
		m_Indexs.emplace_back(index);

		if (m_Indexs.size() >= 2U)
			CreatePath(cutPaths, cp);
	}

	void ForceCreateWhenFinalWasAddedBeforeFirst(PoolOfRecyclables<FacesCutPath>& cutPaths, const std::vector<CutPathElement>& cp)
	{
		auto curSize = m_Indexs.size();
		CreatePath(cutPaths, cp, m_Indexs[curSize - 1U], m_Indexs[curSize - 2U]);
	}

private:
	void CreatePath(PoolOfRecyclables<FacesCutPath>& cutPaths, const std::vector<CutPathElement>& cp)
	{
		auto curSize = m_Indexs.size();
		CreatePath(cutPaths, cp, m_Indexs[curSize - 2U], m_Indexs[curSize - 1U]);
	}

	void CreatePath(PoolOfRecyclables<FacesCutPath>& cutPaths, const std::vector<CutPathElement>& cp, int first, int final)
	{
		auto& fcp = cutPaths.Recycle();
		fcp.Init(first, final, cp);
		m_Paths.emplace_back(&fcp);

		m_MapToFCP->AddPath(*m_Face, cp[first].GetPoint(), fcp);
		m_MapToFCP->AddPath(*m_Face, cp[final].GetPoint(), fcp);
	}

	std::vector<FacesCutPath*> m_Paths;
	std::vector<int> m_Indexs;
	Face* m_Face = nullptr;
	MapToFacesCutPath* m_MapToFCP = nullptr;
};

class CutPathCreator
{
private:
	bool IntersectionIsAlreadyUsed(const EdgeFaceIntersection& inter)
	{
		auto& pe = inter.GetEdge();

		if (pe.HashIsAssigned())
		{
			auto& piercedFaces = m_UsedIntersections->At(pe.GetHash());
			return std::find(piercedFaces.begin(), piercedFaces.end(), &inter.GetFace()) != piercedFaces.end();
		}
		return false;
	}

	bool FindNextWellDefinedIntersection(const std::vector<EdgeFaceIntersection>& intersections, int& index)
	{

	}

	void MapFaceToFCPCollection(Face& f)
	{
		if (!f.HashIsAssigned())
		{
			f.AssignHash();
			m_FacesCutPathCollections->Recycle().Init(f);
		}
	}

	void MapToUsedIntersections(ShapeEdge& piercingEdge)
	{
		if (!piercingEdge.HashIsAssigned())
		{
			piercingEdge.AssignHash();
			m_UsedIntersections->Recycle().clear();
		}
	}

	void ProcessNewCPE(const CutPathElement& cpe, int indexInPath, const std::vector<CutPathElement>& path)
	{
		auto& entered = cpe.GetFaceEntered();
		auto& exited = cpe.GetFaceExited();
		auto& pe = cpe.GetPiercingEdge();

		MapFaceToFCPCollection(entered);
		MapFaceToFCPCollection(exited);

		m_FacesCutPathCollections->At(entered.GetHash()).AddFirst(indexInPath);
		m_FacesCutPathCollections->At(exited.GetHash()).AddFinal(indexInPath, *m_FacesCutPathObjects, path);

		MapToUsedIntersections(pe);
		m_UsedIntersections->At(pe.GetHash()).emplace_back(&cpe.GetPiercedFace());
	}

	void AddNewCPEToPath(const CutPathElement& cpe, std::vector<CutPathElement>& path)
	{
		path.emplace_back(cpe);
		ProcessNewCPE(cpe, path.size() - 1, path);
	}

	Vector3 DirDefinedByFaces(const Face& f1, const Face& f2)
	{
		return Vector3::Cross(f1.GetNormal(), f2.GetNormal()).Normalized();
	}

	Vector3 DirectionOut(const Face& piercedFace, const Face& face, const ShapeEdge& piercingEdge)
	{
		return DirDefinedByFaces(piercedFace, face).InDirectionOf(face.GetEdgeNormal(piercingEdge));
	}

	Vector3 DirectionIn(const Face& piercedFace, const Face& face, const ShapeEdge& piercingEdge)
	{
		return DirDefinedByFaces(piercedFace, face).InDirectionOf(-face.GetEdgeNormal(piercingEdge));
	}

	CutPathElement CreateInitialCPE(const EdgeFaceIntersection& inter)
	{
		auto& piercedFace = inter.GetFace();
		auto& piercingEdge = inter.GetEdge();

		// arbitrarily choose face entered and face exited
		auto& faceEntered = piercingEdge.GetFace1();
		auto& faceExited = piercingEdge.GetFace2();

		auto firstDirFromPrev = DirectionOut(piercedFace, faceExited, piercingEdge);
	}

	CutPathElement CastToNext(const CutPathElement& curr)
	{
		auto& piercedFace = curr.GetPiercedFace();
		auto& faceEntered = curr.GetFaceEntered();

		auto castDir = DirectionIn(piercedFace, faceEntered, curr.GetPiercingEdge());
		auto origin = curr.GetIntPoint();

		auto res0 = piercedFace.CastToEdgeInside(origin, castDir);
		auto res1 = faceEntered.CastToEdgeInside(origin, castDir);

		if (res0.Distance < res1.Distance)
		{

		}
		else
		{

		}
	}

	bool AreEqual(const CutPathElement& cpe1, const CutPathElement& cpe2)
	{
		return (&cpe1.GetPiercedFace() == &cpe2.GetPiercedFace()) && (&cpe1.GetPiercingEdge() == &cpe2.GetPiercingEdge());
	}

	void GeneratePath(const EdgeFaceIntersection& inter)
	{
		auto& path = m_CutPaths->Recycle().clear();

		auto startCpe = CreateInitialCPE(inter);
		auto cpe = CastToNext(startCpe);

		AddNewCPEToPath(startCpe, path);

		while (!AreEqual(startCpe, cpe))
		{
			AddNewCPEToPath(cpe, path);
			cpe = CastToNext(cpe);
		}
	}

public:
	void GeneratePaths(const std::vector<EdgeFaceIntersection>& intersections)
	{
		ObjectWithHash<Face>::ResetNextHashCounter();
		ObjectWithHash<ShapePoint>::ResetNextHashCounter();
		ObjectWithHash<ShapeEdge>::ResetNextHashCounter();

		int index;
		while (FindNextWellDefinedIntersection(intersections, index))
			GeneratePath(intersections[index]);
	}

	const auto& GetFacesCutPathCollections() const
	{
		return *m_FacesCutPathCollections;
	}

	const auto& GetCutPaths() const
	{
		return *m_CutPaths;
	}

private:
	std::unique_ptr<PoolOfRecyclables<std::vector<Face*>>> m_UsedIntersections; // Keyed by piercing edge. Values are the pierced faces.

	std::unique_ptr<PoolOfRecyclables<FacesCutPath>> m_FacesCutPathObjects;
	std::unique_ptr<PoolOfRecyclables<std::vector<CutPathElement>>> m_CutPaths;
	std::unique_ptr<PoolOfRecyclables<FacesCutPaths>> m_FacesCutPathCollections; // keyed by face

	MapToFacesCutPath m_MapToFCPs;
};
