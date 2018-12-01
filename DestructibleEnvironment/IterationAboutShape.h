#pragma once
#include <vector>
#include "Face.h"
#include "ShapeEdge.h"
#include "ShapePoint.h"

class IterationAboutShape
{
private:
	static ShapeEdge& NextEdgeAboutPoint(const ShapePoint& p, const ShapeEdge& curr, const Face& currFace)
	{
		auto indexOfEdgeInCurrFace = curr.GetIndex(currFace);

		if (&p == &curr.GetEnd(currFace))
		{
			return *currFace.GetEdgeObjects()[currFace.NextPointIndex(indexOfEdgeInCurrFace)];
		}
		else if (&p == &curr.GetStart(currFace))
		{
			return *currFace.GetEdgeObjects()[currFace.PreviousPointIndex(indexOfEdgeInCurrFace)];
		}

		assert(false);
		return  *currFace.GetEdgeObjects()[indexOfEdgeInCurrFace];
	}

public:
	static void FindEdgesAndFacesAboutPoint(const ShapePoint& p, ShapeEdge& startEdge, std::vector<ShapeEdge*>& edges, std::vector<Face*>& faces)
	{
		auto f = &startEdge.GetFace1(); // either face is ok.
		auto e = &startEdge;

		do
		{
			edges.emplace_back(e);
			faces.emplace_back(f);

			e = &NextEdgeAboutPoint(p, *e, *f);
			f = &e->GetOther(*f);

		} while (e != &startEdge);
	}
};
