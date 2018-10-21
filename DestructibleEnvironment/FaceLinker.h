#pragma once
#include <array>
#include "Face.h"
#include "PerFaceSplitData.h"
#include "Vector3.h"

class FaceLinker
{
public:
	void SetPerFaceData(std::vector<PerFaceSplitData>& perFaceData)
	{
		m_PerFaceData = &perFaceData;
	}

	template<class... Ts>
	void Link(const std::vector<Face*>& facesToLink, Ts&... searchVectors)
	{
		auto N = sizeof...(Ts);
		std::array<const std::vector<Face*>*, N> potentialNeighbours = { (&searchVectors)... };

		for (auto it = facesToLink.begin(); it != facesToLink.end(); it++)
			LinkFace(**it, potentialNeighbours);
	}

	template<class... Ts>
	void Link(Face& toLink, Ts&... searchVectors) // use this when linking faces detached from cut
	{
		auto N = sizeof...(Ts);
		std::array<const std::vector<Face*>*, N> potentialNeighbours = { (&searchVectors)... };

		LinkFace(toLink, potentialNeighbours);
	}

private:
	static inline float DistanceBetweenEdges(const Vector3& toLinkP0, const Vector3& toLinkP1, const Vector3& otherP0, const Vector3& otherP1)
	{
		// if no overlap return infinity
	}

	static inline bool CanLink(FaceRelationshipWithOtherShape r1, FaceRelationshipWithOtherShape r2)
	{
		if (r1 != r2)
			return (r1 == FaceRelationshipWithOtherShape::Unkown) || (r2 == FaceRelationshipWithOtherShape::Unkown);

		return true;
	}

	template<int N>
	void FindLinkForEdge(Face& toLink, int edgeToLink, const std::array<const std::vector<Face*>*, N>& potentialNeighbours)
	{
		auto& points = toLink.GetCachedPoints();

		auto& p0 = points[edgeToLink];
		auto& p1 = points[(edgeToLink + 1) % points.size()];

		Face* nearestNeighbour = nullptr;
		auto edgeOnNN = -1;
		auto closestDist = MathU::Infinity;

		auto& perFaceData = *m_PerFaceData;
		auto toLinksRelationship = perFaceData[toLink.GetIdForSplitter()].RelationshipWithOtherShape;

		for (int i = 0; i < N; i++)
		{
			auto& faces = *potentialNeighbours[i];

			for (auto itFace = faces.begin(); itFace != faces.end(); itFace++)
			{
				auto& other = **itFace;

				if (!CanLink(toLinksRelationship, perFaceData[other.GetIdForSplitter()].RelationshipWithOtherShape))
					continue;

				auto& otherPoints = other.GetCachedPoints();
				auto otherPointCount = otherPoints.size();

				for (auto j = 0U; j < otherPointCount; j++)
				{
					auto dist = DistanceBetweenEdges(p0, p1, otherPoints[j], otherPoints[(j + 1) % otherPointCount]);

					if (dist < closestDist)
					{
						closestDist = dist;
						nearestNeighbour = &other;
						edgeOnNN = j;
					}
				}
			}
		}

		auto& nnData = perFaceData[nearestNeighbour->GetIdForSplitter()];
		if (nnData.RelationshipWithOtherShape == FaceRelationshipWithOtherShape::Unkown)
		{
			// maybe do a test to check linking is the correct thing to do
		}

		nnData.RelationshipWithOtherShape = toLinksRelationship;
		toLink.AddLink(edgeToLink, nearestNeighbour, edgeOnNN);
	}

	template<int N>
	void LinkFace(Face& toLink, const std::array<const std::vector<Face*>*, N>& potentialNeighbours)
	{
		auto& links = toLink.GetLinkedFaces();

		for (auto i = 0U; i < links.size(); i++)
		{
			if (links.At(i).size() == 0U)
				FindLinkForEdge(toLink, i, potentialNeighbours);
		}
	}

	std::vector<PerFaceSplitData>* m_PerFaceData = nullptr;
};
