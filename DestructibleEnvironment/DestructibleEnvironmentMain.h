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
		DestructibleEnvironmentMain(const std::shared_ptr<DX::DeviceResources>& deviceResources,
			const WindowsInput^ input);

		~DestructibleEnvironmentMain();

		void* operator new(size_t i)
		{
			return _mm_malloc(i, 16);
		}

		void operator delete(void* p)
		{
			_mm_free(p);
		}

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

		// Cached pointer to device resources.
		std::shared_ptr<DX::DeviceResources> m_deviceResources;

		// Rendering loop timer.
		DX::StepTimer m_timer;

		World m_World;
	};
}