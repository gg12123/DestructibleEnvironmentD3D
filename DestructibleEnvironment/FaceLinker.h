#pragma once
#include "Face.h"
#include "PerFaceSplitData.h"

class FaceWithEdge
{
public:
	Face * TheFace;
	int Edge;
};

class FaceLinker
{
public:


private:
	template<int N>
	void FindLinkForEdge(Face& toLink, int edgeToLink, const std::vector<Face*>* potentialNeighbours[N])
	{
		auto& points = toLink.GetCachedPoints();

		auto& p0 = points[edgeToLink];
		auto& p1 = points[(edgeToLink + 1) % points.size()];

		FaceWithEdge neighbour;

		for (int i = 0; i < N; i++)
		{
			auto& faces = *potentialNeighbours[i];

			// find closest and ensure the face relationship with other shape is consistent
		}

		// first check that the neighbours relationship is not inconsistent

		m_PerFaceData[neighbour.TheFace->GetIdForSplitter()].RelationshipWithOtherShape =
			m_PerFaceData[toLink.GetIdForSplitter()].RelationshipWithOtherShape;
	}

	template<int N>
	void LinkFace(Face& toLink, const std::vector<Face*>* potentialNeighbours[N])
	{
		auto& links = toLink.GetLinkedFaces();

		for (auto i = 0U; i < links.size(); i++)
		{
			if (links[i].size() == 0U)
				FindLinkForEdge(toLink, i, potentialNeighbours);
		}
	}

	std::vector<PerFaceSplitData>* m_PerFaceData;
};
