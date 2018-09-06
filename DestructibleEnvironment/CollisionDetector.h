#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "Constants.h"
#include "ArrayWrapper.h"
#include "PoolOfRecyclables.h"
#include "CollisionData.h"
#include "PotentialCollision.h"

class Shape;
class Transform;

class CollisionDetector
{
public:
	CollisionDetector()
	{
		std::function<PotentialCollision()> creator = []()
		{
			return PotentialCollision();
		};

		m_PotentialCollisionPool = std::unique_ptr<PoolOfRecyclables<PotentialCollision>>
			(new PoolOfRecyclables<PotentialCollision>(10, std::move(creator)));
	}

	// Coll normal will point from shape 1 to shape 2. The data must be used befor the next call to this method
	bool FindCollision(Shape& shape1, Shape& shape2, CollisionData& outputData);

private:
	bool FindEdgeCollision(const std::vector<Vector3>& facePoints, const Vector3& faceNormal, const Vector3& edgeP0, const Vector3& edgeP1);
	void FindEdgeCollisions(Shape& shapeFaces, const Vector3& edgeP0, const Vector3& edgeP1);
	void FindEdgeCollisions(Shape& shapeFaces, const std::vector<int>& edges);

	// Use the active Transform - i.e. the one whose space is being worked in
	Vector3 TransformPoint(const Vector3& point);
	Vector3 TransformDirection(const Vector3& dir);
	Vector3 InverseTransformPoint(const Vector3& point);
	Vector3 InverseTransformDirection(const Vector3& dir);

	bool FindBestPotentialCollision(CollisionData& outputData);

	std::unique_ptr<PoolOfRecyclables<PotentialCollision>> m_PotentialCollisionPool;

	ArrayWrapper<Vector3, Constants::MaxNumPoints> m_Shape1TransformedPoints;
	ArrayWrapper<Vector3, Constants::MaxNumPoints> m_Shape2TransformedPoints;

	Shape* m_Shape1;
	Shape* m_Shape2;

	Transform* m_ActiveTransform;
	ArrayWrapper<Vector3, Constants::MaxNumPoints>* m_CurrentOthersPoints;
};