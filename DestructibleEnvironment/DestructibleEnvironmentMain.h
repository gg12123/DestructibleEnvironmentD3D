#pragma once

#include "Common\StepTimer.h"
#include "Common\DeviceResources.h"
#include "World.h"

// Renders Direct2D and 3D content on the screen.
namespace DestructibleEnvironment
{
	class DestructibleEnvironmentMain : public DX::IDeviceNotify
	{
	public:
		DestructibleEnvironmentMain(const std::shared_ptr<DX::DeviceResources>& deviceResources);
		~DestructibleEnvironmentMain();
		void CreateWindowSizeDependentResources();
		void Update();
		void Render();

		// IDeviceNotify
		virtual void OnDeviceLost();
		virtual void OnDeviceRestored();

	private:
		void RegisterEntitiesWithWorld();

		// Cached pointer to device resources.
		std::shared_ptr<DX::DeviceResources> m_deviceResources;

		// Rendering loop timer.
		DX::StepTimer m_timer;

		World m_World;
	};
}