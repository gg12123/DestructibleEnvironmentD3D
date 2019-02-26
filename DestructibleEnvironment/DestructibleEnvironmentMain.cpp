#include "pch.h"
#include "DestructibleEnvironmentMain.h"
#include "Common\DirectXHelper.h"
#include "ShapeProxy.h"
#include "Camera.h"
#include "Light.h"
#include "MathU.h"
#include "DynamicBodyProxy.h"
#include "StaticShapeProxy.h"
#include "Random.h"
#include "ReadOnlyInput.h"
#include "ViewportDimensions.h"
#include "RayShooter.h"
#include "PlaneMesh.h"

using namespace DestructibleEnvironment;
using namespace Windows::Foundation;
using namespace Windows::System::Threading;
using namespace Windows::UI::Core;
using namespace Concurrency;

// Loads and initializes application assets when the application is loaded.
DestructibleEnvironmentMain::DestructibleEnvironmentMain(const std::shared_ptr<DX::DeviceResources>& deviceResources,
	const WindowsInput^ input) :
	m_deviceResources(deviceResources)
{
	// Register to be notified if the Device is lost or recreated
	m_deviceResources->RegisterDeviceNotify(this);

	m_World.Init(m_deviceResources, ReadOnlyInput(input), ViewportDimensions());
	RegisterEntitiesWithWorld();
}

DestructibleEnvironmentMain::~DestructibleEnvironmentMain()
{
	// Deregister device notification
	m_deviceResources->RegisterDeviceNotify(nullptr);
}

// ###################### to be deleted #####################################
static Vector3 bodiesCentre = Vector3(0.0f, 6.0f, 0.0f);
static float bodiesRadius = 20.0f;
static uint16 bodiesCount = 5;
static float sizeMax = 2.0f;
static float sizeMin = 0.5f;

static Vector3 RandDir()
{
	return Vector3(Random::Range(0.1f, 1.0f), Random::Range(0.1f, 1.0f), Random::Range(0.1f, 1.0f)).Normalized();
}

static Quaternion RandRot()
{
	auto r = Quaternion(Random::Range(0.1f, 1.0f), Random::Range(0.1f, 1.0f), Random::Range(0.1f, 1.0f), Random::Range(0.1f, 1.0f));
	r.Normalize();
	return r;
}

static float RandSize()
{
	return Random::Range(sizeMin, sizeMax);
}

static DynamicBodyProxy* CreateBody(const Vector3& pos, const Quaternion& rot, float width, float height)
{
	auto body = new DynamicBodyProxy();
	body->GetTransform().SetPosition(pos);
	body->GetTransform().SetRotation(rot);
	body->SetInitialHeight(height);
	body->SetInitialWidth(width);
	return body;
}

static void CreateRandomBodies(World& world)
{
	std::vector<DynamicBodyProxy*> added;

	for (auto i = 0U; i < bodiesCount; i++)
	{
		auto pos = bodiesCentre + Random::Range(0.0f, bodiesRadius) * RandDir();
		auto validPos = true;

		for (auto it = added.begin(); it != added.end(); it++)
		{
			auto& other = **it;

			auto dist = (pos - other.GetTransform().GetPosition()).Magnitude();

			if (dist < sizeMax)
			{
				validPos = false;
				break;
			}
		}

		if (validPos)
		{
			auto b = CreateBody(pos, RandRot(), RandSize(), RandSize());
			added.emplace_back(b);
			world.RegisterEntity(std::unique_ptr<Entity>(b));
		}
	}
}

void DestructibleEnvironmentMain::RegisterEntitiesWithWorld()
{
	auto bodyPos1 = bodiesCentre + Vector3::Right();
	auto bodyPos2 = bodiesCentre - Vector3::Right();
	auto bodyPos3 = bodiesCentre + Vector3::Up();
	
	//m_World.RegisterEntity(std::unique_ptr<Entity>(CreateBody(bodyPos1, RandRot(), 1.0f, 1.0f)));
	//m_World.RegisterEntity(std::unique_ptr<Entity>(CreateBody(bodyPos2, RandRot(), 1.0f, 1.0f)));
	//m_World.RegisterEntity(std::unique_ptr<Entity>(CreateBody(bodyPos3, RandRot(), 1.0f, 1.0f)));

	m_World.RegisterEntity(std::unique_ptr<Entity>(CreateBody(bodiesCentre,
		RandRot(),
		1.0f, 2.0f)));

	//CreateRandomBodies(m_World);

	auto floor = new StaticShapeProxy();
	auto floorPos = Vector3(0.0f, 0.0f, 0.0f);
	floor->GetTransform().SetPosition(floorPos);
	floor->GetTransform().SetRotation(Quaternion::Identity());
	floor->SetInitialHeight(1.0f);
	floor->SetInitialWidth(20.0f);
	m_World.RegisterEntity(std::unique_ptr<Entity>(floor));

	auto cam = new Camera();
	auto camPos = Vector3(10.0f, 6.0f, 0.0f);
	auto camLookDir = Vector3::Normalize((bodiesCentre + floorPos) / 2.0f - camPos);
	cam->GetTransform().SetPosition(camPos);
	cam->GetTransform().SetRotation(Quaternion::LookRotation(camLookDir));
	cam->SetFov(MathU::ToRadians(45.0f));
	cam->SetFarClip(1000.0f);
	cam->SetNearClip(0.1f);
	m_World.RegisterEntity(std::unique_ptr<Entity>(cam));

	auto light = new Light();
	light->GetTransform().SetPosition(Vector3(15.0f, 15.0f, 2.0f));
	light->GetTransform().SetRotation(Quaternion::Identity());
	m_World.RegisterEntity(std::unique_ptr<Entity>(light));

	auto shooter = new RayShooter();
	shooter->GetTransform().SetPosition(Vector3::Zero());
	shooter->GetTransform().SetRotation(Quaternion::Identity());
	m_World.RegisterEntity(std::unique_ptr<Entity>(shooter));
}

// ###############################################################

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
