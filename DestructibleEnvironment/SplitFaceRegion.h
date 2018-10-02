#pragma once
#include "Polygon2.h"
#include "Polygon2IntersectionFinder.h"
#include "Polygon2Splitter.h"
#include "PoolOfRecyclables.h"
#include "ConvexPolyCreator.h"
#include "ReadOnlyView.h"
#include "FaceRelationshipWithOtherShape.h"

class SplitFaceRegion
{
public:
	void Init(Polygon2& perimPoly, FaceRelationshipWithOtherShape inOrOut)
	{
		m_PerimeterPoly = &perimPoly;
		m_ContainedChild = nullptr;
		m_InOrOut = inOrOut;
		m_HasParent = false;
	}

	FaceRelationshipWithOtherShape GetInOrOut()
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

	auto GetConvexPieces()
	{
		return ReadOnlyView<Polygon2*>(m_ConvexPieces);
	}

	void ClipToContainedChild(Polygon2IntersectionFinder& intFinder, Polygon2Splitter& splitter, PoolOfRecyclables<Polygon2>& polyPool)
	{
		if (m_ContainedChild)
		{
			auto childsPieces = m_ContainedChild->GetConvexPieces();
			for (auto it = childsPieces.Begin(); it != childsPieces.End(); it++)
				SplitToCovexPoly(**it, intFinder, splitter, polyPool);

			m_ContainedChild->ClipToContainedChild(intFinder, splitter, polyPool);
		}
	}

	void SetContainedChild(SplitFaceRegion& c)
	{
		m_ContainedChild = &c;
		c.m_HasParent = true;
	}

	bool HasParent()
	{
		return m_HasParent;
	}

private:
	void SplitToCovexPoly(const Polygon2& splitterPoly, Polygon2IntersectionFinder& intFinder, Polygon2Splitter& splitter, PoolOfRecyclables<Polygon2>& polyPool)
	{
		m_ForOutput.clear();
		for (auto it = m_ConvexPieces.begin(); it != m_ConvexPieces.end(); it++)
		{
			auto& myPoly = **it;

			m_Intersection.Clear();
			if (intFinder.FindIntersection(splitterPoly, myPoly, m_Intersection))
			{
				splitter.Split(myPoly, m_Intersection, m_ForOutput, polyPool);
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
	bool m_HasParent;
};