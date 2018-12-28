#pragma once

class ViewportDimensions
{
public:
	float GetHeight() const
	{
		return GetWindow()->Bounds.Height;
	}

	float GetWidth() const
	{
		return GetWindow()->Bounds.Width;
	}

	float GetAspect() const
	{
		auto b = GetWindow()->Bounds;
		return b.Width / b.Height;
	}

private:
	Windows::UI::Core::CoreWindow^ GetWindow() const
	{
		return Windows::UI::Core::CoreWindow::GetForCurrentThread();
	}
};
