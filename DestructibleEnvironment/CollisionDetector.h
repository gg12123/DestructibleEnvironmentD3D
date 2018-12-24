#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "CleanIntersectionsWrapper.h"
#include "FaceCollision.h"
#include "Constants.h"
#include "TriangleArray.h"

class Shape;
class Transform;

class CollisionDetector
{
public:
	// TODO - the shapes should be const
	bool FindCollision(Shape& shape1, Shape& shape2, std::vector<FaceCollision>& detectedColls, std::vector<EdgeFaceIntersection>& inters);

private:
	void ProcessFaceFaceInteraction(std::vector<FaceCollision>& detectedColls, Face& faceA, Face& faceB);
	void UnAssignHashes(const std::vector<EdgeFaceIntersection>& inters);

	CleanIntersectionsWrapper m_IntersectionFinder;
	TriangleArray<Constants::MaxNumFaces, bool> m_FaceCollisionCreated;
};