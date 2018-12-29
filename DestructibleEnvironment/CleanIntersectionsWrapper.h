#pragma once
#include <array>
#include "CleanIntersectionFinder.h"
#include "Shape.h"
#include "ShapeElementPool.h"
#include "DynamicTriangleArray.h"

class CleanIntersectionsWrapper
{
private:
	template<class T>
	void ReturnToPool(const std::vector<T*> objs)
	{
		for (auto o : objs)
			ShapeElementPool<T>::Return(*o);
	}

	void ClearTransformedShape()
	{
		ReturnToPool(m_TransformedShape.GetPointObjects());
		ReturnToPool(m_TransformedShape.GetEdgeObjects());
		ReturnToPool(m_TransformedShape.GetFaces());

		m_TransformedShape.Clear();
	}

	Face& MakeTransformedFace(const Quaternion& q, const Face& f)
	{
		auto& transFace = FacePool::Take();
		auto& points = f.GetPointObjects();

		transFace.Clear();

		for (auto i = 0u; i < points.size(); i++)
		{
			auto& p0 = *points[i];
			auto& p1 = *points[(i + 1u) % points.size()];

			auto& e = *m_MapToTransformedEdges.Get(p0.GetHash(), p1.GetHash());

			transFace.AddPoint(*m_MapToTransformedPoint[p0.GetHash()], e);
		}

		transFace.SetNormal(q.RotateV(f.GetNormal()));
		return transFace;
	}

	void CreateTransformedFaces(const Quaternion& q, const std::vector<Face*>& faces)
	{
		for (auto f : faces)
			m_TransformedShape.AddFace(MakeTransformedFace(q, *f));
	}

	void CreateTransformedPoints(const Matrix4& M, const std::vector<ShapePoint*>& points)
	{
		for (auto p : points)
		{
			p->TryAssignHash();
			m_MapToTransformedPoint[p->GetHash()] = &PointPool::Take(M * p->GetPoint());
		}
	}

	void CreateTransformedEdges(const std::vector<ShapeEdge*>& edges)
	{
		for (auto e : edges)
		{
			auto origP0Hash = e->GetP0().GetHash();
			auto origP1Hash = e->GetP1().GetHash();

			m_MapToTransformedEdges.Get(origP0Hash, origP1Hash) =
				&EdgePool::Take(*m_MapToTransformedPoint[origP0Hash], *m_MapToTransformedPoint[origP1Hash]);
		}
	}

	void InitTransformedShape(Shape& shapeMaster, Shape& otherShape)
	{
		auto& tMaster = shapeMaster.GetTransform();
		auto& tOther = otherShape.GetTransform();

		auto toMastersSpace = tMaster.GetWorldToLocalMatrix() * tOther.GetLocalToWorldMatrix();
		auto toMastersRot = tMaster.GetWorldToLocalRotation() * tOther.GetLocalToWorldRotation();

		ShapePoint::ResetNextHashCounter();

		CreateTransformedPoints(toMastersSpace, otherShape.GetPointObjects());
		CreateTransformedEdges(otherShape.GetEdgeObjects());
		CreateTransformedFaces(toMastersRot, otherShape.GetFaces());

		for (auto p : otherShape.GetPointObjects())
			p->ResetHash();

		m_TransformedShape.OnAllFacesAdded();
		m_TransformedShape.GetTransform().SetEqualTo(tMaster);
	}

public:
	void FindIntersections(Shape& shape1, Shape& shape2, std::vector<EdgeFaceIntersection>& inters)
	{
		auto& masterShape = shape1;
		auto& tMaster = masterShape.GetTransform();

		m_Master = &masterShape;
		m_Other = &shape2;

		ClearTransformedShape();
		InitTransformedShape(masterShape, shape2);

		m_Loops.clear();
		m_Finder.FindCleanIntersections(masterShape, m_TransformedShape, m_Loops);

		for (auto loop : m_Loops)
		{
			for (auto& inter : loop->GetIntersections())
			{
				inters.emplace_back(EdgeFaceIntersection(inter.GetFace(),
					inter.GetEdge(),
					tMaster.ToWorldPosition(inter.GetIntPoint())));
			}
		}
	}

	Shape& GetOwnerShape(const Face& f) const
	{
		return &f.GetOwnerShape() == &m_TransformedShape ? *m_Other : *m_Master;
	}

private:
	CleanIntersectionFinder m_Finder;
	std::vector<IntersectionLoop*> m_Loops;

	Shape m_TransformedShape;

	Shape* m_Master;
	Shape* m_Other;

	std::array<ShapePoint*, Constants::MaxNumPoints> m_MapToTransformedPoint;
	DynamicTriangleArray<ShapeEdge*> m_MapToTransformedEdges;
};