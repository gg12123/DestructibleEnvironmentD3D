#include "ConvexFaceCreator.h"
#include "Face.h"

void ConvexFaceCreator::CreateFaces(std::vector<Face*>& faces)
{
	auto f = new Face(); // pool
	f->Clear();

	// just assume that the input is convex for now
	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
		f->AddPoint(*it);

	f->SetNormal(m_Normal);
	faces.emplace_back(f);
}

void ConvexFaceCreator::CreateUpsideDownFaces(std::vector<Face*>& faces)
{
	auto f = new Face(); // pool
	f->Clear();

	// just assume that the input is convex for now
	for (auto i = m_Points.size() - 1; i >= 0U; i--)
		f->AddPoint(m_Points[i]);

	f->SetNormal(-m_Normal);
	faces.emplace_back(f);
}