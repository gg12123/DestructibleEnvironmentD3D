#pragma once

#include "Common\StepTimer.h"
#include "Common\DeviceResources.h"
#include "World.h"

// Renders Direct2D and 3D content on the screen.
namespace DestructibleEnvironment
{
	class DestructibleEnvironmentMain : public DX::IDeviceNotify, public AlignedObject16
	{
	public:
		DestructibleEnvironmentMain(const std::shared_ptr<DX::DeviceResources>& deviceResources,
			const WindowsInput^ input);

		~DestructibleEnvironmentMain();

		void CreateWindowSizeDependentResources();
		void Update();
		void Render();

		bool IsReady()
		{
			return m_World.GetRenderer().IsReadyToRender();
		}

		// IDeviceNotify
		virtual void OnDeviceLost();
		virtual void OnDeviceRestored();

	private:
		void RegisterEntitiesWithWorld();
		void CreateStack();

		// Cached pointer to device resources.
		std::shared_ptr<DX::DeviceResources> m_deviceResources;

		// Rendering loop timer.
		DX::StepTimer m_timer;

		World m_World;
	};
}