#pragma once
#include <array>
#include "Shape.h"
#include "Face.h"
#include "Vector3.h"
#include "Transform.h"
#include "Matrix.h"
#include "CubeFacesCreator.h"
#include "Random.h"
#include "Plane.h"

class CutShapeCreator
{
private:
	class FacesCreator
	{
	private:
		void CreateEdge(int p, int pNext)
		{
			auto& edge = EdgePool::Take(*m_Points[p], *m_Points[pNext]);

			m_Edges.Get(p, pNext) = &edge;
			m_Edges.Get(pNext, p) = &edge;
		}

		Face& CreateFace(const std::array<int, 3>& pointIndexes, int planeId) const
		{
			auto& f = FacePool::Take();

			for (auto i = 0U; i < 3; i++)
			{
				auto j = pointIndexes[i];
				auto k = pointIndexes[(i + 1) % 3];

				f.AddPoint(*m_Points[j], *m_Edges.Get(j, k));
			}
			f.CalculateNormalFromPoints(planeId);

			return f;
		}

		void CreatePoints(const Matrix4& M)
		{
			m_Points[0] = &PointPool::Take(M * Vector3(0.0f, 0.0f, 1.0f));

			m_Points[1] = &PointPool::Take(M * Vector3(-1.0f, -1.0f, 0.0f));
			m_Points[2] = &PointPool::Take(M * Vector3(1.0f, -1.0f, 0.0f));
			m_Points[3] = &PointPool::Take(M * Vector3(1.0f, 1.0f, 0.0f));
			m_Points[4] = &PointPool::Take(M * Vector3(-1.0f, 1.0f, 0.0f));

			m_Points[5] = &PointPool::Take(M * Vector3(0.0f, 0.0f, -20.0f));
		}

		void CreateEdges()
		{
			CreateEdge(1, 2);
			CreateEdge(2, 3);
			CreateEdge(3, 4);
			CreateEdge(4, 1);

			CreateEdge(0, 1);
			CreateEdge(0, 2);
			CreateEdge(0, 3);
			CreateEdge(0, 4);

			CreateEdge(5, 1);
			CreateEdge(5, 2);
			CreateEdge(5, 3);
			CreateEdge(5, 4);
		}

		void CreateFaces(Shape& shape, int firstPlaneId)
		{
			auto planeId = firstPlaneId;

			shape.AddFace(CreateFace(m_Face0, planeId));
			planeId++;

			shape.AddFace(CreateFace(m_Face1, planeId));
			planeId++;

			shape.AddFace(CreateFace(m_Face2, planeId));
			planeId++;

			shape.AddFace(CreateFace(m_Face3, planeId));
			planeId++;

			shape.AddFace(CreateFace(m_Face4, planeId));
			planeId++;

			shape.AddFace(CreateFace(m_Face5, planeId));
			planeId++;

			shape.AddFace(CreateFace(m_Face6, planeId));
			planeId++;

			shape.AddFace(CreateFace(m_Face7, planeId));
			planeId++;
		}

	public:
		FacesCreator()
		{
			m_Face0 = { 1, 2, 0 };
			m_Face1 = { 2, 3, 0 };
			m_Face2 = { 3, 4, 0 };
			m_Face3 = { 4, 1, 0 };

			m_Face4 = { 2, 1, 5 };
			m_Face5 = { 3, 2, 5 };
			m_Face6 = { 4, 3, 5 };
			m_Face7 = { 1, 4, 5 };
		}

		void CreateFaces(Shape& shape, const Matrix4& M, int firstPlaneId)
		{
			CreatePoints(M);
			CreateEdges();
			CreateFaces(shape, firstPlaneId);
		}

	private:
		std::array<ShapePoint*, 6> m_Points;
		TwoDArray<6, 6, ShapeEdge*> m_Edges;

		std::array<int, 3> m_Face0;
		std::array<int, 3> m_Face1;
		std::array<int, 3> m_Face2;
		std::array<int, 3> m_Face3;

		std::array<int, 3> m_Face4;
		std::array<int, 3> m_Face5;
		std::array<int, 3> m_Face6;
		std::array<int, 3> m_Face7;
	};

	class ClosestPlanes
	{
	private:
		bool AlreadyGotPlane(int id, int& index) const
		{
			for (auto i = 0; i < MaxNumPlanes; i++)
			{
				if (m_PlanesIds[i] == id)
				{
					index = i;
					return true;
				}
			}
			return false;
		}

	public:
		static constexpr int MaxNumPlanes = 3;

		void Reset()
		{
			for (int i = 0; i < MaxNumPlanes; i++)
			{
				m_Distances[i] = MathU::Infinity;
				m_PlanesIds[i] = -1;
			}
			m_WithinTolPlanes.clear();
		}

		void Update(int id, const Plane& plane, float dist)
		{
			int indexOfExisting;
			if (AlreadyGotPlane(id, indexOfExisting))
			{
				if (dist < m_Distances[indexOfExisting])
				{
					m_Distances[indexOfExisting] = dist;
					m_Planes[indexOfExisting] = plane;
				}
				return;
			}

			for (int i = 0; i < MaxNumPlanes; i++)
			{
				if (dist < m_Distances[i])
				{
					m_Distances[i] = dist;
					m_Planes[i] = plane;
					m_PlanesIds[i] = id;
					return;
				}
			}
		}

		void FinishUpdating(float tol)
		{
			Plane closest;
			auto closestDist = MathU::Infinity;

			for (int i = 0; i < MaxNumPlanes; i++)
			{
				auto dist = m_Distances[i];

				if (dist <= tol)
					m_WithinTolPlanes.emplace_back(m_Planes[i]);

				if (dist < closestDist)
				{
					closestDist = dist;
					closest = m_Planes[i];
				}
			}

			if (m_WithinTolPlanes.size() == 0u)
				m_WithinTolPlanes.emplace_back(closest);
		}

		const auto& GetNearbyPlanes() const
		{
			return m_WithinTolPlanes;
		}

	private:
		std::array<int, MaxNumPlanes> m_PlanesIds;
		std::array<Plane, MaxNumPlanes> m_Planes;
		std::array<float, MaxNumPlanes> m_Distances;

		std::vector<Plane> m_WithinTolPlanes;
	};

	struct PlaneWithPoint
	{
		const Plane ThePlane;
		const Vector3 ThePoint;

		PlaneWithPoint(const Plane& pl, const Vector3& p) : ThePlane(pl), ThePoint(p)
		{
		}
	};

	Vector3 ClosestPointOnFaceToP(const Vector3& p, const Face& f) const
	{
		auto& points = f.GetCachedPoints();
		assert(points.size() == 3u);

		auto& a = points[0];
		auto& b = points[1];
		auto& c = points[2];

		auto ab = b - a;
		auto ac = c - a;
		auto ap = p - a;
		auto d1 = Vector3::Dot(ab, ap);
		auto d2 = Vector3::Dot(ac, ap);

		if (d1 <= 0.0f && d2 <= 0.0f)
			return a;

		auto bp = p - b;
		auto d3 = Vector3::Dot(ab, bp);
		auto d4 = Vector3::Dot(ac, bp);

		if (d3 >= 0.0f && d4 <= d3)
			return b;

		auto vc = d1 * d4 - d3 * d2;

		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
		{
			auto v = d1 / (d1 - d3);
			return (a + v * ab);
		}

		auto cp = p - c;
		auto d5 = Vector3::Dot(ab, cp);
		auto d6 = Vector3::Dot(ac, cp);

		if (d6 >= 0.0f && d5 <= d6)
			return c;

		auto vb = d5 * d2 - d1 * d6;

		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
		{
			auto w = d2 / (d2 - d6);
			return (a + w * ac);
		}

		auto va = d3 * d6 - d5 * d4;

		if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
		{
			auto w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			return (b + w * (c - b));
		}

		auto denom = 1.0f / (va + vb + vc);
		auto v = vb * denom;
		auto w = vc * denom;
		return (a + ab * v + ac * w);
	}

	float DistanceFromPointToFace(const Vector3& p, const Face& f) const
	{
		return (ClosestPointOnFaceToP(p, f) - p).Magnitude();
	}

	void FindNearbyPlanes(const std::vector<Face*>& faces, const Vector3& p)
	{
		m_NearbyPlanes.Reset();

		for (auto f : faces)
		{
			auto dist = DistanceFromPointToFace(p, *f);
			m_NearbyPlanes.Update(f->GetPlaneId(), f->ToPlane(), dist);
		}
		m_NearbyPlanes.FinishUpdating(0.1f);
	}

	PlaneWithPoint CalculateSplitPlane(const Vector3& p) const
	{
		auto& nearbyPlanes = m_NearbyPlanes.GetNearbyPlanes();

		switch (nearbyPlanes.size())
		{
		case 1:
		{
			return PlaneWithPoint(nearbyPlanes[0], p);
		}
		case 2:
		{
			Ray inter;
			if (!nearbyPlanes[0].Intersects(nearbyPlanes[1], inter))
				return PlaneWithPoint(nearbyPlanes[0], p);

			auto p0 = inter.ClosestPointOnRayAsInfLine(p);
			auto n = (nearbyPlanes[0].GetNormal() + nearbyPlanes[1].GetNormal()).Normalized();

			return PlaneWithPoint(Plane(n, p0), p0);
		}
		case 3:
		{
			Vector3 inter;
			if (!nearbyPlanes[0].Intersects(nearbyPlanes[1], nearbyPlanes[2], inter))
				PlaneWithPoint(nearbyPlanes[0], p);

			return PlaneWithPoint(Plane((nearbyPlanes[0].GetNormal() + nearbyPlanes[1].GetNormal() + nearbyPlanes[2].GetNormal()).Normalized(),
				inter), inter);
		}
		default:
		{
			assert(false);
			return PlaneWithPoint(Plane(), Vector3());
		}
		}
	}

	Matrix4 CalculateCutShapesTransform(Transform& toSplitsTransform, const PlaneWithPoint& splitPlane) const
	{
		static constexpr auto sXY = 10.0f;
		static constexpr auto sZ = 3.0f;
		auto S = Matrix4::FromScale(sXY, sXY, sZ);

		auto& n = splitPlane.ThePlane.GetNormal();
		auto R = Matrix4::FromRotation(Quaternion::LookRotation(-n, Vector3::OrthogonalDirection(n))); // could have some randomnes for up
		
		auto P = splitPlane.ThePoint;
		auto D = 0.05f;// 0.5f * P.Magnitude(); // could have some randomnes

		auto T = Matrix4::FromTranslation(P + (sZ - D) * n);

		return T * R * S;
	}

	//Matrix4 CalculateCutShapesTransform(Transform& toSplitsTransform, const Vector3& splitPoint, const Vector3& splitNormal, Quaternion& q) const
	//{
	//	auto sZ = 100.0f;
	//	auto r = 0.0f; // Random::Range(0.0f, 0.5f);
	//
	//	auto A = r * splitPoint;
	//	auto C = A + sZ * splitNormal;
	//
	//	q = Quaternion::LookRotation(-splitNormal, Vector3::OrthogonalDirection(splitNormal));
	//
	//	return Matrix4::FromTranslation(C) * Matrix4::FromRotation(q) * Matrix4::FromScale(sZ, sZ, sZ);
	//}

public:
	static constexpr auto NumPlanesInCutShape = 4;

	// Split point should be on the surface of one of the faces.
	Shape & Create(Shape& toSplit, const Vector3& splitPointWorld, int firstPlaneId)
	{
		auto& toSplitsTransform = toSplit.GetTransform();

		m_CutShape.Clear();
		m_CutShape.GetTransform().SetEqualTo(toSplitsTransform);

		auto splitPoint = toSplitsTransform.ToLocalPosition(splitPointWorld);
		FindNearbyPlanes(toSplit.GetFaces(), splitPoint);
		auto splitPlane = CalculateSplitPlane(splitPoint);

		auto M = CalculateCutShapesTransform(toSplitsTransform, splitPlane);

		m_FacesCreator.CreateFaces(m_CutShape, M, firstPlaneId);

		m_CutShape.OnAllFacesAdded();

		return m_CutShape;
	}

private:
	Shape m_CutShape;

	ClosestPlanes m_NearbyPlanes;
	FacesCreator m_FacesCreator;
};