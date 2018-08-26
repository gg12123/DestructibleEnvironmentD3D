#pragma once

class Constants
{
public:
	static int constexpr MaxPointsForSplit = 30;
	static int constexpr MaxNumPoints = 60; // assume that splitting cannot double the number of points

	static int constexpr MaxNumFaces = MaxNumPoints; // assume 3 points per face and each point attached to 3 faces
	static int constexpr MaxNumVerts = MaxNumFaces * 4; // one vert for each point on the face, plus one for a centre point
	static int constexpr MaxNumTris = MaxNumFaces * 3; // one for each point on face
	static int constexpr MaxNumIndicies = MaxNumTris * 3;

	static int constexpr MaxNumShapes = 200;
};