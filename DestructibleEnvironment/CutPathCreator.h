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
#include "PiercedOnlyFaceHandler.h"

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

	bool GeneratePath(const IntersectionLoop& loop)
	{
		auto& path = m_CutPaths->Recycle();
		path.clear();

		auto c = loop.GetCount();

		auto& firstFacePierced = loop.GetPiercedFace(0);
		auto& firstFaceExited = loop.GetFaceExited(0);

		auto onlyOnePiercedFace = true;

		for (auto i = 0; i < c; i++)
		{
			auto& pf = loop.GetPiercedFace(i);
			AddNewCPEToPath(CutPathElement(loop.GetFaceExited(i), pf, loop.GetPiercingEdge(i), loop.GetIntPoint(i)), path);

			if (&pf != &loop.GetPiercedFace(MathU::Max(i - 1, 0)))
				onlyOnePiercedFace = false;
		}

		if (!onlyOnePiercedFace)
		{
			m_FacesCutPathCollections->At(firstFacePierced.GetHash()).ForceCreateWhenFinalWasAddedBeforeFirst(*m_FacesCutPathObjects, path);
			m_FacesCutPathCollections->At(firstFaceExited.GetHash()).ForceCreateWhenFinalWasAddedBeforeFirst(*m_FacesCutPathObjects, path);

			return true;
		}

		return false;
	}

	auto TotalIntersectionCount(const std::vector<IntersectionLoop*>& intersectionLoops)
	{
		auto c = 0;
		for (auto loop : intersectionLoops)
			c += loop->GetCount();

		return c;
	}

public:
	bool GeneratePaths(const std::vector<IntersectionLoop*>& intersectionLoops, PiercedOnlyFaceHandler::PiercedFace& piercedOnlyFace)
	{
		// Resize the pools of recyclables now so that
		// they do not resize during the path generation, which
		// leaves pointers dangling.

		m_CutPaths->Reserve(intersectionLoops.size());

		auto c = TotalIntersectionCount(intersectionLoops);
		m_FacesCutPathCollections->Reserve(c);
		m_FacesCutPathObjects->Reserve(c);

		m_CutPaths->Reset();
		m_FacesCutPathObjects->Reset();
		m_FacesCutPathCollections->Reset();

		for (auto loop : intersectionLoops)
		{
			if (!GeneratePath(*loop))
			{
				piercedOnlyFace = PiercedOnlyFaceHandler::PiercedFace(loop->GetPiercedFace(0), loop->IntersectionsCentre());
				return false;
			}
		}
		return true;
	}

	const auto& GetFacesCutPathCollections() const
	{
		return *m_FacesCutPathCollections;
	}

	auto& GetCutPaths() const
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

	CutPathCreator()
	{
		m_FacesCutPathObjects =
			std::unique_ptr<PoolOfRecyclables<FacesCutPath>>(new PoolOfRecyclables<FacesCutPath>(20));

		m_CutPaths =
			std::unique_ptr<PoolOfRecyclables<std::vector<CutPathElement>>>(new PoolOfRecyclables<std::vector<CutPathElement>>(3));

		auto c = [this]()
		{
			return FacesCutPaths(m_MapToFCPs);
		};

		m_FacesCutPathCollections =
			std::unique_ptr<PoolOfRecyclables<FacesCutPaths>>(new PoolOfRecyclables<FacesCutPaths>(20, c));
	}

private:
	std::unique_ptr<PoolOfRecyclables<FacesCutPath>> m_FacesCutPathObjects;
	std::unique_ptr<PoolOfRecyclables<std::vector<CutPathElement>>> m_CutPaths;
	std::unique_ptr<PoolOfRecyclables<FacesCutPaths>> m_FacesCutPathCollections; // keyed by face

	MapToFacesCutPath m_MapToFCPs;
};
