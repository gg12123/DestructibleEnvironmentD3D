#pragma once

#include "Vector.h"
#include "Point.h"
#include "ShapeEdge.h"
#include "Face.h"

class FinalFaceCreator
{
public:
	FinalFaceCreator()
	{
	}

	void AddEdge(ShapeEdge& e)
	{
		e.GetEdgeP1()->AddLink(*e.GetEdgeP2());
		e.GetEdgeP2()->AddLink(*e.GetEdgeP1());

		m_Head = e.GetEdgeP1();
	}

	Face& CreateFace(const Vector3& finalFaceNormal)
	{
		auto prev = CalculatePrev(finalFaceNormal);
		auto curr = m_Head;

		auto face = new Face(); // Get from pool
		auto& points = face->GetPoints();

		do
		{
			points.push_back(curr);

			auto next = curr->NextLinkedPoint(*prev);

			prev = curr;
			curr = next;

		} while (curr != m_Head);

		face->SetNormal(finalFaceNormal);

		return *face;
	}

private:
	Point* CalculatePrev(const Vector3& finalFaceNormal)
	{
		auto P0 = m_Head->GetLinkedPoint1()->GetPoint();
		auto P1 = m_Head->GetPoint();
		auto P2 = m_Head->GetLinkedPoint2()->GetPoint();

		auto e01 = Vector3::Normalize(P1 - P0);
		auto e12 = Vector3::Normalize(P2 - P1);

		auto c = Vector3::Cross(e01, e12);

		return Vector3::Dot(c, finalFaceNormal) > 0.0f ? m_Head->GetLinkedPoint1() : m_Head->GetLinkedPoint2();
	}

	Point * m_Head = nullptr;
};
