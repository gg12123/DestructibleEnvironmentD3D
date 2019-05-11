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
#include "SimdExperiments.h"
#include "WindmillMotor.h"

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
	static float sizeMax = 2.0f;
	static float sizeMin = 0.5f;
	return Random::Range(sizeMin, sizeMax);
}

static DynamicBodyProxy* CreateBody(const Vector3& pos, const Quaternion& rot,
	const std::vector<SubShapeData>& subShapes,
	const std::vector<SubShapeLink>& links)
{
	auto body = new DynamicBodyProxy();
	body->GetTransform().SetPosition(pos);
	body->GetTransform().SetRotation(rot);

	for (auto& ss : subShapes)
		body->AddSubShapeData(ss);

	for (auto& l : links)
		body->AddSubShapeLink(l);

	return body;
}

void DestructibleEnvironmentMain::CreateStack()
{
	auto pos = Vector3(0.0f, 1.0f, 0.0f);

	auto x = 2.0f * pos;

	for (auto i = 0; i < 5; i++)
	{
		m_World.RegisterEntity(std::unique_ptr<Entity>(CreateBody(pos, Quaternion::Identity(),
			{ SubShapeData(Vector3::Zero(), Vector3(1.0f, 1.0f, 1.0f)) },
			{})));

		pos += Vector3::Up();
	}
}

void DestructibleEnvironmentMain::CreateWindmill(StaticShapeProxy& floor, float floorHeight, float height, float x, float z, const Vector3& faceDir)
{
	static constexpr auto baseWidth = 0.5f;
	static constexpr auto millWidth = 1.0f;
	static constexpr auto bladeLength = 1.0f;
	auto baseHeight = height;

	// Create the base
	auto base = new DynamicBodyProxy();
	base->GetTransform().SetPosition(Vector3(x, floorHeight + baseHeight / 2.0f, z));
	base->GetTransform().SetRotation(Quaternion::Identity());
	base->AddSubShapeData(SubShapeData(Vector3::Zero(), Vector3(baseWidth, baseHeight, baseWidth)));
	m_World.RegisterEntity(std::unique_ptr<Entity>(base));

	// Joint the base to the floor
	auto jointTransformBase = Matrix4::FromTranslation(base->GetTransform().GetPosition());
	auto& anchorPhysBase = floor.GetStaticBody();
	auto& bodyPhysBase = base->GetPhysicsBody();
	auto joint = Joint(jointTransformBase, anchorPhysBase, bodyPhysBase, *anchorPhysBase.GetSubShapes()[0], *bodyPhysBase.GetSubShapes()[0],
		{ true, true, true });
	m_World.GetPhysics().AddJoint(joint);

	// Create the mill
	auto mill = new DynamicBodyProxy();
	mill->GetTransform().SetPosition(Vector3(x, floorHeight + baseHeight, z) + (baseWidth / 2.0f + millWidth / 2.0f + 0.1f) * faceDir);
	mill->GetTransform().SetRotation(Quaternion::Identity());
	mill->AddSubShapeData(SubShapeData(Vector3::Zero(), Vector3(millWidth, millWidth, millWidth)));
	mill->AddSubShapeData(SubShapeData((millWidth / 2.0f + bladeLength / 2.0f) * Vector3::Foward(), Vector3(millWidth / 2.0f, millWidth / 2.0f, bladeLength)));
	mill->AddSubShapeData(SubShapeData(-(millWidth / 2.0f + bladeLength / 2.0f) * Vector3::Foward(), Vector3(millWidth / 2.0f, millWidth / 2.0f, bladeLength)));
	mill->AddSubShapeLink(SubShapeLink(0, 1));
	mill->AddSubShapeLink(SubShapeLink(0, 2));
	m_World.RegisterEntity(std::unique_ptr<Entity>(mill));

	// Join the mill to the base
	auto jointTransform = Matrix4::FromTranslation(Vector3(x, floorHeight + baseHeight, z) + (baseWidth / 2.0f) * faceDir);
	auto& anchorPhys = base->GetPhysicsBody();
	auto& bodyPhys = mill->GetPhysicsBody();
	auto joint1 = Joint(jointTransform, anchorPhys, bodyPhys, *anchorPhys.GetSubShapes()[0], *bodyPhys.GetSubShapes()[0],
		{ false, true, true });
	m_World.GetPhysics().AddJoint(joint1);

	// Create the motor
	auto mot = new WindmillMotor(*base);
	m_World.RegisterEntity(std::unique_ptr<Entity>(mot));
}

void DestructibleEnvironmentMain::RegisterEntitiesWithWorld()
{
	auto millHeight = 4.0f;
	auto anchorPos = Vector3(0.0f, millHeight, 0.0f);

	auto floor = new StaticShapeProxy();
	auto floorPos = Vector3(0.0f, 0.0f, 0.0f);
	floor->GetTransform().SetPosition(floorPos);
	floor->GetTransform().SetRotation(Quaternion::Identity());
	floor->AddSubShapeData(SubShapeData(Vector3::Zero(), Vector3(20.0f, 1.0f, 20.0f)));
	m_World.RegisterEntity(std::unique_ptr<Entity>(floor));

	CreateWindmill(*floor, 0.5f, millHeight, 0.0f, 0.0f, Vector3::Right());

	auto cam = new Camera();
	auto camPos = Vector3(10.0f, 6.0f, 0.0f);
	auto camLookDir = Vector3::Normalize((anchorPos + floorPos) / 2.0f - camPos);
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
