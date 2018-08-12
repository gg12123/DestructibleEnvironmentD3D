#pragma once

class Renderer;

class IMeshRenderer
{
public:
	virtual void Render(Renderer& renderer) = 0;
};
