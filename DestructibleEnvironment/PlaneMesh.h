#pragma once
#include "DynamicMesh.h"
#include "Vector2.h"
#include "Vector3.h"

class PlaneMesh : public DynamicMesh
{
public:
	PlaneMesh(const Vector2& size) : m_Size(size)
	{
	}

protected:
	void Awake() override
	{
		auto& verts = MapVertexBuffer();

		auto n = Vector3::Foward();

		auto halfXSize = m_Size.x / 2.0f;
		auto halfYSize = m_Size.y / 2.0f;

		verts.emplace_back(Vertex(Vector3(halfXSize, halfYSize, 0.0f), n));
		verts.emplace_back(Vertex(Vector3(-halfXSize, halfYSize, 0.0f), n));
		verts.emplace_back(Vertex(Vector3(-halfXSize, -halfYSize, 0.0f), n));
		verts.emplace_back(Vertex(Vector3(halfXSize, -halfYSize, 0.0f), n));

		UnMapVertexBuffer();

		auto& indexs = MapIndexBuffer();

		indexs.emplace_back(0u);
		indexs.emplace_back(1u);
		indexs.emplace_back(3u);

		indexs.emplace_back(1u);
		indexs.emplace_back(2u);
		indexs.emplace_back(3u);

		UnMapIndexBuffer();
	}

private:
	Vector2 m_Size;
};
