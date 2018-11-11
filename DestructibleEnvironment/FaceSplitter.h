#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "FacesCutPaths.h"
#include "FacesPointsIterator.h"

class FaceSplitter
{
private:
	void InitFaces(const FacesCutPaths& paths, std::vector<Face*>& newFaces)
	{
		auto newFace = new Face(); //TODO - pool
		m_Iterator.InitFaces(paths.GetFace(), *newFace);
		newFaces.emplace_back(newFace);
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
				InitFaces(paths, newFaces);
				m_Iterator.IterateInside(fcp);
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
				InitFaces(paths, newFaces);
				m_Iterator.IterateOutside(fcp);
			}
		}
	}

	void InitMaps(const MapToShapePointOnReversedFace& map, const MapToNewEdges& edgeMap, const MapToFacesCutPath& mapToFcp)
	{
		m_Iterator.InitMaps(map, edgeMap, mapToFcp);
	}
private:
	FacesPointsIterator m_Iterator;
};