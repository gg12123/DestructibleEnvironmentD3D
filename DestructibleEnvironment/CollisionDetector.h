#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "Constants.h"
#include "ArrayWrapper.h"
#include "PoolOfRecyclables.h"
#include "CollisionData.h"

class Shape;
class Transform;

class CollisionDetector
{
public:
	CollisionDetector()
	{
		std::function<std::unique_ptr<CollisionData>()> creator = []()
		{
			return std::unique_ptr<CollisionData>(new CollisionData());
		};

		m_DataPool = std::unique_ptr<PoolOfRecyclables<std::unique_ptr<CollisionData>>>
			(new PoolOfRecyclables<std::unique_ptr<CollisionData>>(10, std::move(creator)));
	}

	// Coll normal will point from shape 1 to shape 2. The data must be used befor the next call to this method
	CollisionData * FindCollision(Shape& shape1, Shape& shape2);

private:
	void FindPointFaceCollision(const std::vector<Vector3>& faceNormals, const std::vector<Vector3>& faceP0s, const Vector3& point);
	void FindPointFaceCollisions(Shape& shapeFaces, ArrayWrapper<Vector3, Constants::MaxNumPoints>&  points);

	void FindEdgeCollision(const std::vector<Vector3>& facePoints, const Vector3& faceNormal, const Vector3& edgeP0, const Vector3& edgeP1);
	void FindEdgeCollisions(Shape& shapeFaces, const Vector3& edgeP0, const Vector3& edgeP1);
	void FindEdgeCollisions(Shape& shapeFaces, ArrayWrapper<Vector3, Constants::MaxNumPoints>&  points, const std::vector<int>& edges);

	// Use the active Transform - i.e. the one whose space is being worked in
	Vector3 TransformPoint(const Vector3& point);
	Vector3 TransformDirection(const Vector3& dir);
	Vector3 InverseTransformPoint(const Vector3& point);
	Vector3 InverseTransformDirection(const Vector3& dir);

	void FinalisePointFaceCollisions(Shape& shapeFaces);
	void FinaliseEdgeCollisions();

	CollisionData* FinalCollisionData();

	std::vector<CollisionData*> m_FoundCollisions;
	std::vector<CollisionData*> m_FinalisedFoundCollisions;
	std::unique_ptr<PoolOfRecyclables<std::unique_ptr<CollisionData>>> m_DataPool;

	ArrayWrapper<Vector3, Constants::MaxNumPoints> m_Shape1TransformedPoints;
	ArrayWrapper<Vector3, Constants::MaxNumPoints> m_Shape2TransformedPoints;

	Shape* m_Shape1;
	Shape* m_Shape2;
	Transform* m_ActiveTransform;
};