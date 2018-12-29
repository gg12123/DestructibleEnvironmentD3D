#pragma once

class Constants
{
public:
	static int constexpr AverageNumFaces = 100;
	static int constexpr AverageNumPoints = AverageNumFaces;
	
	static int constexpr AverageNumVerts = AverageNumFaces * 3;
	static int constexpr AverageNumTris = AverageNumFaces;
	static int constexpr AverageNumIndicies = AverageNumTris * 3;

	static int constexpr MaxNumShapes = 200;
};