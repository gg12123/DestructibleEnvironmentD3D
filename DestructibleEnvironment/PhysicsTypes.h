#pragma once

enum class PhysicsObjectType
{
	Rigidbody,
	Trigger,
	CharController,
	Static
};

enum class PhysicsGeometryType
{
	ConvexMesh,
	Sphere,
	Capsule
};

enum class PhysicsObjectComparisonType
{
	ContainsTrigger,
	ContainsRbAndPhysical,
	CharControllerCharController,
	CharControllerStatic
};