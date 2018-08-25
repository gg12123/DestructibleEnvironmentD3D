#include "pch.h"
#include "DestructibleEnvironmentMain.h"
#include "Common\DirectXHelper.h"
#include "ShapeProxy.h"
#include "Camera.h"
#include "Light.h"
#include "MathUtils.h"

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
	auto shape = new ShapeProxy();
	shape->GetTransform().SetPosition(Vector3(0.0f, 0.0f, 0.0f));
	shape->GetTransform().SetRotation(Quaternion::Identity());
	shape->SetInitialHeight(2.0f);
	shape->SetInitialWidth(1.0f);
	m_World.RegisterEntity(*shape);

	auto cam = new Camera();
	cam->GetTransform().SetPosition(Vector3(10.0f, 0.0f, 0.0f));
	cam->GetTransform().SetRotation(Quaternion::LookRotation(-Vector3::Right()));
	cam->SetFov(MathUtils::ToRadians(45.0f));
	cam->SetFarClip(1000.0f);
	cam->SetNearClip(0.1f);
	m_World.RegisterEntity(*cam);

	auto light = new Light();
	light->GetTransform().SetPosition(Vector3(5.0f, 5.0f, 2.0f));
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
