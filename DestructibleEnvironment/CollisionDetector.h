#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "ShapeConstants.h"
#include "ArrayWrapper.h"

class CollisionData
{
public:
	Vector3 Normal;
	Vector3 Position;
	float Penetration;
};

class CollisionDataPool
{
public:

	void Reset()
	{
		m_Curr = 0;
	}

	CollisionData & GetNextData()
	{
		if (m_Curr == m_Data.size())
			m_Data.emplace_back(std::unique_ptr<CollisionData>(new CollisionData()));

		m_Curr++;
		return *m_Data[m_Curr - 1];
	}

private:
	std::vector<std::unique_ptr<CollisionData>> m_Data;
	int m_Curr;
};

class Shape;
class Transform;

class CollisionDetector
{
public:
	// Coll normal will point from shape 1 to shape 2
	CollisionData * FindCollision(Shape& shape1, Shape& shape2);

private:
	void FindPointFaceCollision(const std::vector<Vector3>& faceNormals, const std::vector<Vector3>& faceP0s, const Vector3& point);
	void FindPointFaceCollisions(Shape& shapeFaces, ArrayWrapper<Vector3, ShapeConstants::MaxNumPoints>&  points);

	void FindEdgeCollision(const std::vector<Vector3>& facePoints, const Vector3& faceNormal, const Vector3& edgeP0, const Vector3& edgeP1);
	void FindEdgeCollisions(Shape& shapeFaces, const Vector3& edgeP0, const Vector3& edgeP1);
	void FindEdgeCollisions(Shape& shapeFaces, ArrayWrapper<Vector3, ShapeConstants::MaxNumPoints>&  points, const std::vector<int>& edges);

	// Use the active Transform - i.e. the one whose space is being worked in
	Vector3 TransformPoint(const Vector3& point);
	Vector3 TransformDirection(const Vector3& dir);
	Vector3 InverseTransformPoint(const Vector3& point);
	Vector3 InverseTransformDirection(const Vector3& dir);

	CollisionData& FinalCollisionData();

	std::vector<CollisionData*> m_FoundCollisions;
	CollisionDataPool m_DataPool;

	ArrayWrapper<Vector3, ShapeConstants::MaxNumPoints> m_Shape1TransformedPoints;
	ArrayWrapper<Vector3, ShapeConstants::MaxNumPoints> m_Shape2TransformedPoints;

	Shape* m_Shape1;
	Shape* m_Shape2;
	Transform* m_ActiveTransform;
};