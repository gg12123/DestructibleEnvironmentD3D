#pragma once
#include "Shape.h"
#include "ShapeProxy.h"
#include "Vector3.h"
#include "Point.h"
#include "Face.h"
#include "ShapeEdge.h"

class InitialShapeCreator
{
public:
	static void Create(Shape& shape, float width, float height, const Transform& transform)
	{
		auto w = width / 2.0f;;
		auto h = height / 2.0f;

		auto s = Vector3(w, h, w);

		// pool
		auto P0 = new Point(Vector3(s.x, s.y, s.z));
		auto P1 = new Point(Vector3(-s.x, s.y, s.z));
		auto P2 = new Point(Vector3(-s.x, s.y, -s.z));
		auto P3 = new Point(Vector3(s.x, s.y, -s.z));

		auto P4 = new Point(Vector3(s.x, -s.y, s.z));
		auto P5 = new Point(Vector3(-s.x, -s.y, s.z));
		auto P6 = new Point(Vector3(-s.x, -s.y, -s.z));
		auto P7 = new Point(Vector3(s.x, -s.y, -s.z));

		// points

		shape.AddPoint(*P0);
		shape.AddPoint(*P1);
		shape.AddPoint(*P2);
		shape.AddPoint(*P3);
		shape.AddPoint(*P4);
		shape.AddPoint(*P5);
		shape.AddPoint(*P6);
		shape.AddPoint(*P7);

		// edges

		shape.AddEdge(*(new ShapeEdge(*P1, *P0)));
		shape.AddEdge(*(new ShapeEdge(*P0, *P3)));
		shape.AddEdge(*(new ShapeEdge(*P3, *P2)));
		shape.AddEdge(*(new ShapeEdge(*P2, *P1)));

		shape.AddEdge(*(new ShapeEdge(*P0, *P4)));
		shape.AddEdge(*(new ShapeEdge(*P3, *P7)));
		shape.AddEdge(*(new ShapeEdge(*P2, *P6)));
		shape.AddEdge(*(new ShapeEdge(*P1, *P5)));

		shape.AddEdge(*(new ShapeEdge(*P4, *P7)));
		shape.AddEdge(*(new ShapeEdge(*P7, *P6)));
		shape.AddEdge(*(new ShapeEdge(*P6, *P5)));
		shape.AddEdge(*(new ShapeEdge(*P5, *P4)));

		// faces

		shape.GetFaces().push_back(MakeFace(P1, P0, P3, P2));
		shape.GetFaces().push_back(MakeFace(P0, P4, P7, P3));
		shape.GetFaces().push_back(MakeFace(P3, P7, P6, P2));
		shape.GetFaces().push_back(MakeFace(P2, P6, P5, P1));
		shape.GetFaces().push_back(MakeFace(P1, P5, P4, P0));
		shape.GetFaces().push_back(MakeFace(P4, P5, P6, P7));

		// init

		shape.CentreAndCache();
		shape.InitRequiredVertAndIndexCounts();

		auto& faces = shape.GetFaces();
		for (auto it = faces.begin(); it != faces.end(); it++)
			(*it)->CachePoints(shape.GetCachedFaceNormals(), shape.GetCachedFaceP0s());

		shape.GetTransform().SetEqualTo(transform);
	}

private:

	static Face* MakeFace(Point* p1, Point* p2, Point* p3, Point* p4)
	{
		auto f = new Face(); // pool

		f->GetPoints().push_back(p1);
		f->GetPoints().push_back(p2);
		f->GetPoints().push_back(p3);
		f->GetPoints().push_back(p4);

		f->SetNormal(Normal(p1, p2, p3));

		return f;
	}

	static Vector3 Normal(Point* p1, Point* p2, Point* p3)
	{
		auto n = Vector3::Normalize(Vector3::Cross(p3->GetPoint() - p2->GetPoint(), p1->GetPoint() - p2->GetPoint()));

		if (Vector3::Dot(p1->GetPoint(), n) < 0.0f)
			n *= -1.0f;

		return n;
	}
};
