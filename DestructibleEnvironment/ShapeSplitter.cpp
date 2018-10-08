#include "ShapeSplitter.h"
#include "Shape.h"
#include "Face.h"

static bool FaceIsInsideShape(Face& face, Shape& shape)
{
	auto& tFace = face.GetShape().GetTransform();
	auto& tShape = shape.GetTransform();

	auto c = PointInPolyCase::OnBoundary;

	auto& facePoints = face.GetCachedPoints();
	auto it = facePoints.begin();

	while ((c == PointInPolyCase::OnBoundary) && (it != facePoints.end()))
	{
		auto testPoint = tShape.ToLocalPosition(tFace.ToWorldPosition(*it));
		c = shape.PointIsInsideShape(testPoint);
		it++;
	}
	assert(c != PointInPolyCase::OnBoundary);
	return c == PointInPolyCase::Inside;
}

static Face& CreateReversedFace(const Face& f)
{
	auto reversed = new Face(); // pool

	auto& points = f.GetCachedPoints();
	
	for (int i = points.size() - 1; i >= 0; i--)
		reversed->AddPoint(points[i]);

	return *reversed;
}

void ShapeSplitter::CreateNewFaces()
{
	m_NewInIntersectionFaces.clear();
	m_NewOutsideFaces.clear();
	m_FaceSplitter.SetOutputs(m_NewOutsideFaces, m_NewInIntersectionFaces);

	auto& orignalsFaces = m_OriginalShape->GetFaces();
	for (auto it = orignalsFaces.begin(); it != orignalsFaces.end(); it++)
	{
		auto& f = **it;

		if (f.HasRegisteredIntersections())
		{
			m_FaceSplitter.SplitOriginalShapesFace(**it);
		}
		else
		{
			auto& toAddTo = FaceIsInsideShape(f, *m_CutShape) ? m_NewInIntersectionFaces : m_NewOutsideFaces;
			toAddTo.emplace_back(&f);
		}
	}

	auto& cutShapesFaces = m_CutShape->GetFaces();
	for (auto it = cutShapesFaces.begin(); it != cutShapesFaces.end(); it++)
	{
		auto& f = **it;

		if (f.HasRegisteredIntersections())
		{
			m_FaceSplitter.SplitCutShapesFace(**it);
		}
		else if (FaceIsInsideShape(f, *m_OriginalShape))
		{
			m_NewInIntersectionFaces.emplace_back(&f);
			m_NewOutsideFaces.emplace_back(&CreateReversedFace(f));
		}
	}
}

void ShapeSplitter::SwapInNewFaces(Shape& newShape)
{
	auto& refTransform = m_OriginalShape->GetTransform();

	newShape.SwapInNewFaces(m_NewInIntersectionFaces, refTransform);
	m_OriginalShape->SwapInNewFaces(m_NewOutsideFaces, refTransform);
}

void ShapeSplitter::Split(const Vector3& splitPoint, const Vector3& splitNormal, Shape& orginalShape, Shape& newShape)
{
	m_OriginalShape = &orginalShape;

	PositionCutShape(splitPoint, splitNormal);
	RegisterIntersections();
	CreateNewFaces();
	SwapInNewFaces(newShape);
}

static void ClearIntersections(const std::vector<Face*>& faces)
{
	for (auto it = faces.begin(); it != faces.end(); it++)
		(*it)->ClearRegisteredIntersections();
}

void ShapeSplitter::ClearRegisteredIntersections()
{
	ClearIntersections(m_CutShape->GetFaces());
	ClearIntersections(m_OriginalShape->GetFaces());
}

void ShapeSplitter::RegisterIntersections()
{
	ClearRegisteredIntersections();

	m_FaceFaceIntersections.clear();
	m_IntersectionFinder.FindFaceFaceIntersections(*m_OriginalShape, *m_CutShape, m_FaceFaceIntersections);

	for (auto it = m_FaceFaceIntersections.begin(); it != m_FaceFaceIntersections.end(); it++)
	{
		auto& x = *it;

		x.Face1->RegisterIntersection(x);
		x.Face2->RegisterIntersection(x);
	}
}