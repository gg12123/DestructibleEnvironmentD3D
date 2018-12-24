#include "pch.h"
#include "CollisionDetector.h"
#include "Shape.h"
#include "MathU.h"
#include "Transform.h"
#include "ShapeEdge.h"

void CollisionDetector::ProcessFaceFaceInteraction(std::vector<FaceCollision>& detectedColls, Face& faceA, Face& faceB)
{
	faceA.TryAssignHash();
	faceB.TryAssignHash();

	if (!m_FaceCollisionCreated.Get(faceA.GetHash(), faceB.GetHash()))
	{
		detectedColls.emplace_back(FaceCollision(faceA, faceB));

		m_FaceCollisionCreated.Get(faceA.GetHash(), faceB.GetHash()) = true;
	}
}

void CollisionDetector::UnAssignHashes(const std::vector<EdgeFaceIntersection>& inters)
{
	for (auto& inter : inters)
	{
		auto& e = inter.GetEdge();
		auto& f = inter.GetFace();

		f.ResetHash();
		e.GetFace1().ResetHash();
		e.GetFace2().ResetHash();
	}
}

bool CollisionDetector::FindCollision(Shape& shape1, Shape& shape2, std::vector<FaceCollision>& detectedColls, std::vector<EdgeFaceIntersection>& inters)
{
	detectedColls.clear();
	inters.clear();

	m_IntersectionFinder.FindEdgeFaceIntersections(shape1, shape2, inters);

	if (inters.size() > 0U)
	{
		m_FaceCollisionCreated.Clear(false);

		Face::ResetNextHashCounter();

		for (auto& inter : inters)
		{
			auto& e = inter.GetEdge();
			auto& f = inter.GetFace();

			ProcessFaceFaceInteraction(detectedColls, f, e.GetFace1());
			ProcessFaceFaceInteraction(detectedColls, f, e.GetFace2());
		}

		UnAssignHashes(inters);
		return true;
	}
	return false;
}