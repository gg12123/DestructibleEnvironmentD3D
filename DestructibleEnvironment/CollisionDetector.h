#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "FaceCollision.h"
#include "Constants.h"
#include "TriangleArray.h"
#include "IntersectionFinder.h"

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

	IntersectionFinder m_IntersectionFinder;
	TriangleArray<Constants::MaxNumFaces, bool> m_FaceCollisionCreated;
};