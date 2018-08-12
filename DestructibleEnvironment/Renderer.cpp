#include "pch.h"
#include "Renderer.h"
#include "IMeshRenderer.h"

void Renderer::BindVertexBuffer(ID3D11Buffer* buffer, unsigned int stride, unsigned int offset)
{
	m_Context->IASetVertexBuffers(0, 1, &buffer, &stride, &offset);
}

void Renderer::BindIndexBuffer(ID3D11Buffer* buffer)
{
	m_Context->IASetIndexBuffer(buffer, DXGI_FORMAT_R32_UINT, 0);
}

void Renderer::SetObjectToWorld(const Transform& transform)
{
	// use the obj to world constant buffer
}

void Renderer::Draw(int indexCount)
{
	m_Context->DrawIndexed(indexCount, 0, 0);
}

void Renderer::Render()
{
	// set acitive shader
	// set camera and light params
	// set input layout

	m_Context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	for (auto it = m_Renderers.begin(); it != m_Renderers.end(); it++)
		(*it)->Render(*this);
}