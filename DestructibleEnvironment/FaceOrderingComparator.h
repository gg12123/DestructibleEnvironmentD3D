#pragma once
#include "Face.h"
#include "Vector3.h"
#include "Vector2.h"

enum class FaceOrder
{
	FaceAOverlaysB,
	FaceBOverlaysA,
	NoOverlay
};

class FaceOrderingComparator
{
private:

public:
	FaceOrder CompareFaces(const Face& faceA, const Face& faceB, const Vector3& rayDir)
	{
		// Face A overlays face B if all seperating planes imply face A is hit first by the ray.
		// Face B overlays face A if all seperating planes imply face B is hit first by the ray.
		// If not all planes imply the same result, the 2 faces do not overlay.
	}

private:
};
