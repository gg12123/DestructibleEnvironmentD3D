#include "pch.h"
#include "DestructibleEnvironmentMain.h"
#include "Common\DirectXHelper.h"
#include "ShapeProxy.h"
#include "Camera.h"
#include "Light.h"
#include "MathUtils.h"
#include "DynamicBodyProxy.h"
#include "StaticShapeProxy.h"

using namespace DestructibleEnvironment;
using namespace Windows::Foundation;
using namespace Windows::System::Threading;
using namespace Concurrency;

// Loads and initializes application assets when the application is loaded.
DestructibleEnvironmentMain::DestructibleEnvironmentMain(const std::shared_ptr<DX::DeviceResources>& deviceResources) :
	m_deviceResources(deviceResources)
{
	// Register to be notified if the Device is lost or recreated
	m_deviceResources->RegisterDeviceNotify(this);

	m_World.Init(m_deviceResources);
	RegisterEntitiesWithWorld();
}

DestructibleEnvironmentMain::~DestructibleEnvironmentMain()
{
	// Deregister device notification
	m_deviceResources->RegisterDeviceNotify(nullptr);
}

void DestructibleEnvironmentMain::RegisterEntitiesWithWorld()
{
	auto body = new DynamicBodyProxy();
	auto bodyPos = Vector3(0.0f, 6.0f, 0.0f);
	body->GetTransform().SetPosition(bodyPos);
	auto bodyForward = Vector3(1.0f, 0.0f, 0.0f).Normalized();
	auto bodyUp = Vector3(0.0f, 1.0f, 1.0f).Normalized();
	body->GetTransform().SetRotation(Quaternion::LookRotation(bodyForward, bodyUp));
	body->SetInitialHeight(3.0f);
	body->SetInitialWidth(1.0f);
	body->SetMass(0.1f);
	body->SetDrag(0.7f);
	body->SetAngularDrag(0.7f);
	m_World.RegisterEntity(*body);

	auto floor = new StaticShapeProxy();
	auto floorPos = Vector3(0.0f, 0.0f, 0.0f);
	floor->GetTransform().SetPosition(floorPos);
	floor->GetTransform().SetRotation(Quaternion::Identity());
	floor->SetInitialHeight(1.0f);
	floor->SetInitialWidth(10.0f);
	m_World.RegisterEntity(*floor);

	auto cam = new Camera();
	auto camPos = Vector3(10.0f, 6.0f, 0.0f);
	auto camLookDir = Vector3::Normalize((bodyPos + floorPos) / 2.0f - camPos);
	cam->GetTransform().SetPosition(camPos);
	cam->GetTransform().SetRotation(Quaternion::LookRotation(camLookDir));
	cam->SetFov(MathUtils::ToRadians(45.0f));
	cam->SetFarClip(1000.0f);
	cam->SetNearClip(0.1f);
	m_World.RegisterEntity(*cam);

	auto light = new Light();
	light->GetTransform().SetPosition(Vector3(15.0f, 15.0f, 2.0f));
	light->GetTransform().SetRotation(Quaternion::Identity());
	m_World.RegisterEntity(*light);
}

// Updates application state when the window size changes (e.g. device orientation change)
void DestructibleEnvironmentMain::CreateWindowSizeDependentResources() 
{
	// TODO: Replace this with the size-dependent initialization of your app's content.
}

// Updates the application state once per frame.
void DestructibleEnvironmentMain::Update() 
{
	m_World.Update();
}

// Renders the current frame according to the current application state.
// Returns true if the frame was rendered and is ready to be displayed.
void DestructibleEnvironmentMain::Render()
{
	auto context = m_deviceResources->GetD3DDeviceContext();

	// Reset the viewport to target the whole screen.
	auto viewport = m_deviceResources->GetScreenViewport();
	context->RSSetViewports(1, &viewport);

	// Reset render targets to the screen.
	ID3D11RenderTargetView *const targets[1] = { m_deviceResources->GetBackBufferRenderTargetView() };
	context->OMSetRenderTargets(1, targets, m_deviceResources->GetDepthStencilView());

	// Clear the back buffer and depth stencil view.
	context->ClearRenderTargetView(m_deviceResources->GetBackBufferRenderTargetView(), DirectX::Colors::CornflowerBlue);
	context->ClearDepthStencilView(m_deviceResources->GetDepthStencilView(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	m_World.Render();
}

// Notifies renderers that device resources need to be released.
void DestructibleEnvironmentMain::OnDeviceLost()
{
	// TODO - work out what I should do here

	//m_sceneRenderer->ReleaseDeviceDependentResources();
	//m_fpsTextRenderer->ReleaseDeviceDependentResources();
}

// Notifies renderers that device resources may now be recreated.
void DestructibleEnvironmentMain::OnDeviceRestored()
{
	// TODO - work out what I should do here

	//m_sceneRenderer->CreateDeviceDependentResources();
	//m_fpsTextRenderer->CreateDeviceDependentResources();
	CreateWindowSizeDependentResources();
}
