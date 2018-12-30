#include "pch.h"
#include "Camera.h"
#include "World.h"
#include "MathU.h"

void Camera::Awake()
{
	GetWorld().GetRenderer().SetCamera(*this);
}

Matrix4 Camera::CalcuateVPMatrix()
{
	auto aspect = GetWorld().GetRenderer().GetViewportDimensions().GetAspect();

	return Matrix4::Perspective(m_Fov, aspect, m_NearClip, m_FarClip) *
		GetTransform().GetWorldToLocalMatrix();
}

Ray Camera::ScreenPointToRay(const Vector2& p)
{
	static constexpr auto d = 10.0f;

	auto& viewDims = GetWorld().GetRenderer().GetViewportDimensions();
	auto theta = 0.5f * m_Fov;

	auto h = d * tanf(theta);
	auto w = h * viewDims.GetAspect();

	auto halfSw = viewDims.GetWidth() / 2.0f;
	auto halfSh = viewDims.GetHeight() / 2.0f;

	auto pX = (p.x - halfSw) / halfSw;
	auto pY = (p.y - halfSh) / halfSh;

	auto dirCamSpace = (Vector3(pX * w, pY * h, d)).Normalized();

	auto& t = GetTransform();
	return Ray(t.GetPosition(), t.ToWorldDirection(dirCamSpace));
}