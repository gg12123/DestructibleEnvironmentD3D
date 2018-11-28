#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "FacesCutPaths.h"
#include "FacesPointsIterator.h"
#include "ConcaveFace.h"
#include "ShapeEdgesCreator.h"

class FaceSplitter
{
private:
	void InitFaces(const FacesCutPaths& paths)
	{
		m_ConcaveFace.Clear();
		m_Iterator.InitFaces(paths.GetFace(), m_ConcaveFace);
	}

	void CreateTriangleFaces(const FacesCutPaths& paths, std::vector<Face*>& newFaces)
	{
		m_ConcaveFace.Triangulate(paths.GetFace(), newFaces);
	}

public:
	// TODO - remove the repetition.
	void SpitInside(const FacesCutPaths& paths, std::vector<Face*>& newFaces)
	{
		auto& fcps = paths.GetPaths();

		for (auto it = fcps.begin(); it != fcps.end(); it++)
		{
			auto& fcp = **it;

			if (!fcp.BeenUsedToGenInsideFace())
			{
				InitFaces(paths);
				m_Iterator.IterateInside(fcp);
				CreateTriangleFaces(paths, newFaces);
			}
		}
	}

	// TODO - remove the repetition.
	void SplitOutside(const FacesCutPaths& paths, std::vector<Face*>& newFaces)
	{
		auto& fcps = paths.GetPaths();

		for (auto it = fcps.begin(); it != fcps.end(); it++)
		{
			auto& fcp = **it;

			if (!fcp.BeenUsedToGenOutsideFace())
			{
				InitFaces(paths);
				m_Iterator.IterateOutside(fcp);
				CreateTriangleFaces(paths, newFaces);
			}
		}
	}

	void InitMaps(const MapToShapePointOnReversedFace& map, ShapeEdgesCreator& edgesCreator, const MapToFacesCutPath& mapToFcp)
	{
		m_Iterator.InitMaps(map, edgesCreator.GetMapToNewEdges(), mapToFcp);
	}
private:
	FacesPointsIterator m_Iterator;
	ConcaveFace m_ConcaveFace;
};