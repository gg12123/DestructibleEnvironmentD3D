#pragma once
#include "Shape.h"
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "EdgeEdgeRelationship.h"

class CleanIntersectionFinder
{
private:
	const EdgeEdgeRelationship& GetEdgeEdgeRelationship(const ShapeEdge& edgeFaces, const ShapeEdge& piercingEdge) const
	{

	}

	void DeterminePointPlaneRelationships(const std::vector<ShapePoint*>& points, const std::vector<Face*>& faces)
	{

	}

	void DetermineEdgeEdgeRelationships(const std::vector<ShapeEdge*>& piercedFacesEdges, const std::vector<ShapeEdge*>& piercingEdges)
	{

	}

	void FindIntersection(const Face& face, const ShapeEdge& piercingEdge)
	{
		auto& facesEdges = face.GetEdgeObjects();

		for (auto& faceEdge : facesEdges)
		{
			if (!GetEdgeEdgeRelationship(*faceEdge, piercingEdge).ImpliesIntersectionFor(face))
				return;
		}

		// register the intersection
	}

	void FindIntersections(const std::vector<Face*>& faces, const std::vector<ShapeEdge*>& piercingEdges)
	{
		for (auto& f : faces)
			for (auto& e : piercingEdges)
				FindIntersection(*f, *e);
	}

	bool ValidateIntersections()
	{
	}

	void LinkIntersections()
	{

	}

public:
	// Both shapes must have the same local space.
	bool FindCleanIntersections(const Shape& shapeEdges, const Shape& shapeFaces)
	{
		// assign hashes

		DeterminePointPlaneRelationships(shapeEdges.GetPointObjects(), shapeFaces.GetFaces());
		DetermineEdgeEdgeRelationships(shapeFaces.GetEdgeObjects(), shapeEdges.GetEdgeObjects());
		FindIntersections(shapeFaces.GetFaces(), shapeEdges.GetEdgeObjects());

		auto valid = false;

		if (ValidateIntersections())
		{
			LinkIntersections();
			valid = true;
		}

		// un-assign hashes
		return valid;
	}
};
