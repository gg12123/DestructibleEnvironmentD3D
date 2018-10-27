#pragma once
#include "Polygon2.h"
#include "Polygon2IntersectionFinder.h"
#include "Polygon2Splitter.h"
#include "PoolOfRecyclables.h"
#include "ConvexPolyCreator.h"
#include "FaceRelationshipWithOtherShape.h"
#include "Face.h"

class SplitFaceRegion
{
public:
	void Init(Polygon2& perimPoly, FaceRelationshipWithOtherShape inOrOut)
	{
		m_PerimeterPoly = &perimPoly;
		m_ContainedChild = nullptr;
		m_InOrOut = inOrOut;
	}

	const Polygon2& GetPerimeterPoly() const
	{
		return *m_PerimeterPoly;
	}

	FaceRelationshipWithOtherShape GetInOrOut() const
	{
		if (m_InOrOut == FaceRelationshipWithOtherShape::Unkown)
		{
			auto childs = m_ContainedChild->GetInOrOut();
			assert(childs != FaceRelationshipWithOtherShape::Unkown);

			return childs == FaceRelationshipWithOtherShape::InIntersection ? 
				FaceRelationshipWithOtherShape::NotInIntersection : FaceRelationshipWithOtherShape::InIntersection;
		}
		return m_InOrOut;
	}

	void GenerateInitialConvexPieces(PoolOfRecyclables<Polygon2>& polyPool, ConvexPolyCreator& convexCreator)
	{
		m_ConvexPieces.clear();
		convexCreator.ToConvexPieces(*m_PerimeterPoly, m_ConvexPieces, polyPool);
	}

	const auto& GetConvexPieces()
	{
		return m_ConvexPieces;
	}

	void CreateFaces(std::vector<Face*>& faces, const Face& original) const
	{
		CreateFacesImp<false>(faces, original);
	}

	void CreateReversedFaces(std::vector<Face*>& faces, const Face& original) const
	{
		CreateFacesImp<true>(faces, original);
	}

	void ClipToContainedChild(Polygon2IntersectionFinder& intFinder, Polygon2Splitter& splitter, PoolOfRecyclables<Polygon2>& polyPool)
	{
		if (m_ContainedChild)
		{
			auto& childsPieces = m_ContainedChild->GetConvexPieces();
			for (auto it = childsPieces.begin(); it != childsPieces.end(); it++)
				SplitToCovexPoly(**it, intFinder, splitter, polyPool);

			m_ContainedChild->ClipToContainedChild(intFinder, splitter, polyPool);
		}
	}

	void SetContainedChild(SplitFaceRegion& c)
	{
		m_ContainedChild = &c;
	}

private:
	template<bool reversed> // not reversed
	void AddPointsToFace(Face& face, const std::vector<Vector2>& points, const Face& original)
	{
		for (auto it2 = points.begin(); it2 != points.end(); it2++)
			face.AddPoint(original.ToShapeSpacePosition(*it2));
	}

	template<> // reversed
	void AddPointsToFace<true>(Face& face, const std::vector<Vector2>& points, const Face& original)
	{
		for (int i = points.size() - 1; i >= 0; i--)
			face.AddPoint(original.ToShapeSpacePosition(points[i]));
	}

	template<bool reversed> // not reversed
	void SetFaceNormal(Face& face, const Face& original)
	{
		face.SetNormal(original.GetNormal());
	}

	template<> // reversed
	void SetFaceNormal<true>(Face& face, const Face& original)
	{
		face.SetNormal(-original.GetNormal());
	}

	template<bool reversed>
	void CreateFacesImp(std::vector<Face*>& faces, const Face& original) const
	{
		for (auto it = m_ConvexPieces.begin(); it != m_ConvexPieces.end(); it++)
		{
			auto f = new Face(); // use pool
			f->Clear();

			auto& points = (*it)->GetPoints();

			AddPointsToFace<reversed>(*f, points, original);
			SetFaceNormal<reversed>(*f, original);

			faces.emplace_back(f);
		}
	}

	void GenerateMapToOverlaidOuterEdge(const Polygon2& outerPoly, const Polygon2& intersectionPoly)
	{
		m_MapToOverlaidOuterEdge.clear();
		for (int i = 0; i < m_Intersection.GetPointCount(); i++)
			m_MapToOverlaidOuterEdge.emplace_back(-1);

		auto lineIntsProcessed = 0;
		auto currLineIntIndex = 0;

		while (lineIntsProcessed < m_LineIntDetails.size())
		{
			auto& intDetais = m_LineIntDetails[currLineIntIndex];
			if (intDetais.EnteringPoly == &outerPoly)
			{
				auto indexInIntersection = intDetais.PointIndexInIntersection;
				auto indexInOuter = intDetais.EdgeOnEnteringPoly;

				auto endIndexInIntersection = m_LineIntDetails[(currLineIntIndex + 1) & m_LineIntDetails.size()].PointIndexInIntersection;

				while (indexInIntersection != endIndexInIntersection)
				{
					m_MapToOverlaidOuterEdge[indexInIntersection] = indexInOuter;

					indexInOuter = outerPoly.NextIndex(indexInOuter);
					indexInIntersection = intersectionPoly.NextIndex(indexInIntersection);
				}

				lineIntsProcessed += 2;
				currLineIntIndex = (currLineIntIndex + 2) % m_LineIntDetails.size();
			}
			else
			{
				currLineIntIndex = (currLineIntIndex + 1) % m_LineIntDetails.size();
			}
		}
	}

	void SplitToCovexPoly(const Polygon2& splitterPoly, Polygon2IntersectionFinder& intFinder, Polygon2Splitter& splitter, PoolOfRecyclables<Polygon2>& polyPool)
	{
		m_ForOutput.clear();
		for (auto it = m_ConvexPieces.begin(); it != m_ConvexPieces.end(); it++)
		{
			auto& myPoly = **it;

			m_Intersection.Clear();
			m_LineIntDetails.clear();
			if (intFinder.FindIntersection(splitterPoly, myPoly, m_Intersection, m_LineIntDetails))
			{
				GenerateMapToOverlaidOuterEdge(myPoly, m_Intersection);
				splitter.Split(myPoly, m_Intersection, m_MapToOverlaidOuterEdge, m_ForOutput, polyPool);
			}
			else
			{
				m_ForOutput.emplace_back(&myPoly);
			}
		}
		m_ConvexPieces.swap(m_ForOutput);
	}

	Polygon2 m_Intersection;
	SplitFaceRegion* m_ContainedChild = nullptr;

	Polygon2* m_PerimeterPoly = nullptr;
	std::vector<Polygon2*> m_ConvexPieces;
	std::vector<Polygon2*> m_ForOutput;

	FaceRelationshipWithOtherShape m_InOrOut;

	std::vector<LineIntersectionDetails> m_LineIntDetails;
	std::vector<int> m_MapToOverlaidOuterEdge;
};