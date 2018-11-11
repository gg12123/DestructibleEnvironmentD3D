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
