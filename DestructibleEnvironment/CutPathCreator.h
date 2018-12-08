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
#include "FacesCutPaths.h"
#include "CutPathIntersectionsHandler.h"
#include "IntersectionLoop.h"

class CutPathCreator
{
private:
	void MapFaceToFCPCollection(Face& f)
	{
		if (!f.HashIsAssigned())
		{
			f.AssignHash();
			m_FacesCutPathCollections->Recycle().Init(f);
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
	}

	void AddNewCPEToPath(const CutPathElement& cpe, std::vector<CutPathElement>& path)
	{
		path.emplace_back(cpe);
		ProcessNewCPE(cpe, path.size() - 1, path);
	}

	void GeneratePath(const IntersectionLoop& loop)
	{
		auto& path = m_CutPaths->Recycle();
		path.clear();

		auto c = loop.GetCount();

		for (auto i = 0; i < c; i++)
			AddNewCPEToPath(CutPathElement(loop.GetFaceExited(i), loop.GetPiercedFace(i), loop.GetPiercingEdge(i)), path);
	}

public:
	void GeneratePaths(std::vector<IntersectionLoop*>& intersectionLoops)
	{
		m_CutPaths->Reset();

		for (auto loop : intersectionLoops)
			GeneratePath(*loop);
	}

	const auto& GetFacesCutPathCollections() const
	{
		return *m_FacesCutPathCollections;
	}

	const auto& GetCutPaths() const
	{
		return *m_CutPaths;
	}

	bool FaceIsSplit(const Face& f)
	{
		if (f.HashIsAssigned() && (f.GetHash() < m_FacesCutPathCollections->NumRecycled()))
			return true;

		return false;
	}

	const auto& GetMapToFacesCutPaths()
	{
		return m_MapToFCPs;
	}

private:
	std::unique_ptr<PoolOfRecyclables<FacesCutPath>> m_FacesCutPathObjects;
	std::unique_ptr<PoolOfRecyclables<std::vector<CutPathElement>>> m_CutPaths;
	std::unique_ptr<PoolOfRecyclables<FacesCutPaths>> m_FacesCutPathCollections; // keyed by face

	MapToFacesCutPath m_MapToFCPs;
};
