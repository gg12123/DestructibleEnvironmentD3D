# Destructible object physics engine

This is a physics engine that is optimised for destructible objects. The main features are described below.

- Algorithm used for breaking shape geometry into fragments in real time. Used for destroying shapes. This is defined in ShapeDestructor.h.
- Collision detection is done using a hierarchical grid in the broad phase (see HGrid.h) and GJK/EPA in the narrow phase (see CollisionDetector.h).
- The engine uses a sequential impulses solver (see SequentialImpulsesSolver.h). The inputs to the solver are the constraints and the islands. Constraints are defined in Constraints.h and the islands are created in Islands.h.
- There is one joint type (see Joint.h) that can be constrained in various ways to achieve different joint behaviours. For example, if rotation is constrained about two axes, the joint will behave like a hinge joint.
